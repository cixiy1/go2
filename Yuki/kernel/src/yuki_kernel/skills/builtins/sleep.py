from time import sleep


class Sleep:
    @staticmethod
    def output(time: int) -> str:
        if not time:
            return "错误：请输入等待时间"
        return f"等待：{time}s"

    @staticmethod
    def run_sleep(time: int) -> str:
        if not time:
            return "错误：请输入等待时间"
        sleep(time)
        return f"等待完成：{time}s"
