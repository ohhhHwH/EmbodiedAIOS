

import asyncio
from weather import *  

def limited_print(*args, **kwargs):
    # 遍历所有参数，将每个参数转换为字符串并截取前20个字符
    truncated_args = [str(arg)[:100] for arg in args]
    # 调用原生的 print 函数打印处理后的参数
    print(*truncated_args, **kwargs)
    

async def test():
    try:
        CA_weather = await get_alerts("CA")
        print("get_alerts('CA') result:")
        limited_print(CA_weather)
        location_weather = await get_forecast(34.0522, -118.2437)  # Los Angeles coordinates
        print("get_forecast(34.0522, -118.2437) result:")
        limited_print(location_weather)
    finally:
        pass
    
def main():
    print("Hello from weather!")
    
if __name__ == "__main__":
    # main()
    import sys
    asyncio.run(test())
