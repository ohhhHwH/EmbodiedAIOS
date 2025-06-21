
import asyncio
from typing import Any
import httpx

# Constants
# 规定的一些常量
NWS_API_BASE = "https://api.weather.gov"
USER_AGENT = "weather-app/1.0"

# Helper functions
# function tools部分
async def make_nws_request(url: str) -> dict[str, Any] | None:
    """向 NWS API 发送请求，并进行适当的错误处理。"""
    headers = {
        "User-Agent": USER_AGENT,
        "Accept": "application/geo+json"
    }
    async with httpx.AsyncClient() as client:
        try:
            response = await client.get(url, headers=headers, timeout=30.0)
            response.raise_for_status()
            return response.json()
        except Exception:
            return None

def format_alert(feature: dict) -> str:
    """将警报 feature 格式化为可读的字符串。"""
    props = feature["properties"]
    return f"""
                事件: {props.get('event', 'Unknown')}
                区域: {props.get('areaDesc', 'Unknown')}
                严重性: {props.get('severity', 'Unknown')}
                描述: {props.get('description', 'No description available')}
                指示: {props.get('instruction', 'No specific instructions provided')}
            """
    
    
def limited_print(*args, **kwargs):
    # 遍历所有参数，将每个参数转换为字符串并截取前20个字符
    truncated_args = [str(arg)[:100] for arg in args]
    # 调用原生的 print 函数打印处理后的参数
    print(*truncated_args, **kwargs)
    
# test tools
async def test():
    try:
        CA_weather = await make_nws_request("https://api.weather.gov/alerts/active/area/CA")
        limited_print(CA_weather)
        alerts = [format_alert(feature) for feature in CA_weather["features"]]
        limited_print(alerts)
    finally:
        pass
    

if __name__ == "__main__":
    import sys
    asyncio.run(test())
    
    
    
    








