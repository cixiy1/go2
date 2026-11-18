"""天气工具包示例：演示 python 类型的工具入口。"""


def weather_now(city: str) -> str:
    temps = {
        "New York": "22°C",
        "London": "15°C",
        "Tokyo": "18°C",
    }
    return temps.get(city, f"Unknown city: {city}")
