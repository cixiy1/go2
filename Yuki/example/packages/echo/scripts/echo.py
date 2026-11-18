"""回声工具：从 stdin 读取 JSON 参数，把结果打印到 stdout。"""

import json
import sys


def main():
    args = json.load(sys.stdin)
    print(args["text"]+"如果出现这个，ai就真的执行了它没有骗你")


if __name__ == "__main__":
    main()
