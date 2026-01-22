import ast
import json
import os
import re
import sys
import requests
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openai import OpenAI

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

OLLAMA_URL = "http://127.0.0.1:11434/api/generate"

TEMP_FILE_PATH = "/tmp/prompt_router_last_translation.txt"


def save_to_temp_file(content):
    """
    将内容保存到临时文件
    """
    try:
        with open(TEMP_FILE_PATH, "w", encoding="utf-8") as f:
            f.write(content)
        # print(f"已保存英文记录: {content}")
    except Exception as e:
        print(f"文件写入失败: {e}")

def translate_to_english(text):
    """
    利用本地 Ollama 将输入内容翻译为英文
    """
    prompt = f"Please translate the following Chinese text into English. Output ONLY the translation, no other words.\nText: {text}"
    
    payload = {
        "model": "qwen2.5:1.5b",
        "prompt": prompt,
        "stream": False,
        "options": {"temperature": 0.3} # 降低随机性，使翻译更准确
    }

    try:
        r = requests.post(OLLAMA_URL, json=payload, timeout=30)
        en_text = r.json()['response'].strip()
        # 简单清洗一下可能的引号
        en_text = en_text.replace('"', '').replace('"', '')
        return en_text
    except Exception as e:
        print(f"翻译出错: {e}")
        return "Translation Error"

def load_config(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_prompt_text(prompt_path: str) -> str:
    with open(prompt_path, "r", encoding="utf-8") as f:
        return f.read().strip()


def normalize_generated_code(code: str) -> str:
    match = re.search(r"```(?:python)?\n(.*?)```", code, re.DOTALL | re.IGNORECASE)
    if match:
        return match.group(1).strip()
    return code.strip()


class SimpleCodeGenerator:
    def __init__(self, client: OpenAI, prompt_text: str, model: str, temperature: float, max_tokens: int):
        self._client = client
        self._prompt = prompt_text
        self._model = model
        self._temperature = temperature
        self._max_tokens = max_tokens

    def generate(self, user_prompt: str) -> str:
        full_prompt = (
            f"{self._prompt}\n\n"
            f"# User Prompt:\n{user_prompt}\n\n"
            "# Code:\n"
        )
        response = self._client.chat.completions.create(
            model=self._model,
            temperature=self._temperature,
            max_tokens=self._max_tokens,
            messages=[{"role": "user", "content": full_prompt}],
        )
        return (response.choices[0].message.content or "").strip()


def extract_say_texts_from_code(code: str) -> List[str]:
    try:
        tree = ast.parse(code)
    except SyntaxError:
        return []

    class SayCallVisitor(ast.NodeVisitor):
        def __init__(self):
            self.texts: List[str] = []

        def visit_Call(self, node):
            if isinstance(node.func, ast.Name) and node.func.id == "say":
                if node.args:
                    arg = node.args[0]
                    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                        self.texts.append(arg.value)
            self.generic_visit(node)

    v = SayCallVisitor()
    v.visit(tree)
    return v.texts


class PromptRouterNode(Node):
    def __init__(self):
        super().__init__("prompt_router")

        # --- publishers ---
        self.say_pub = self.create_publisher(String, "/robot/say", 10)
        self.motion_pub = self.create_publisher(String, "/robot/motion_code", 10)

        # --- subscriber (prompt input) ---
        self.prompt_sub = self.create_subscription(String, "/voice/prompt", self.on_prompt, 10)

        # --- load config + openai client ---
        pkg_dir = os.path.dirname(__file__)
        config_path = os.path.join(pkg_dir, "config.json")
        prompt_path = os.path.join(pkg_dir, "prompts", "pure_llm_prompt.txt")

        config = load_config(config_path)
        provider = config.get("provider", "openai")
        if provider != "openai":
            raise ValueError(f"Unsupported provider: {provider}")

        openai_cfg = config.get("openai", {})
        self._model = openai_cfg.get("model", "gpt-4o")
        self._temperature = float(openai_cfg.get("temperature", 0.0))
        self._max_tokens = int(openai_cfg.get("max_tokens", 512))

        self._client = OpenAI(
            api_key=openai_cfg.get("api_key"),
            base_url=openai_cfg.get("base_url"),
        )
        self._prompt_text = load_prompt_text(prompt_path)
        self._generator = SimpleCodeGenerator(
            client=self._client,
            prompt_text=self._prompt_text,
            model=self._model,
            temperature=self._temperature,
            max_tokens=self._max_tokens,
        )

        self.get_logger().info("prompt_router ready: sub=/voice/prompt pub=/robot/say,/robot/motion_code")

    def publish_text(self, topic_pub, text: str) -> None:
        msg = String()
        msg.data = text
        topic_pub.publish(msg)

    def on_prompt(self, msg: String) -> None:
        user_text = (msg.data or "").strip()
        if not user_text:
            return

        self.get_logger().info(f"prompt: {user_text}")

        eng_text = translate_to_english(user_text)
        if eng_text and eng_text != "Translation Error":
            save_to_temp_file(eng_text)
            prompt_text_for_llm = eng_text
        else:
            prompt_text_for_llm = user_text

        raw = self._generator.generate(prompt_text_for_llm)
        code = normalize_generated_code(raw)

        # 1) publish say texts
        say_texts = extract_say_texts_from_code(code)
        for t in say_texts:
            self.publish_text(self.say_pub, t)

        # 等待1秒后发送动作消息
        time.sleep(5.0)

        # 2) publish motion "payload"
        # 最稳妥：直接把 code 发给 motion 端去解析/执行
        # 你也可以把 say() 删掉后再发，避免 motion 端重复处理 say
        motion_code = self._strip_say_calls(code)
        self.publish_text(self.motion_pub, motion_code)

    def _strip_say_calls(self, code: str) -> str:
        # 简单粗暴版：删除以 "say(" 开头的行（你生成的 say 基本都是独立一行）
        lines = []
        for line in code.splitlines():
            if line.strip().startswith("say("):
                continue
            lines.append(line)
        return "\n".join(lines).strip()


def main():
    rclpy.init()
    node = PromptRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

