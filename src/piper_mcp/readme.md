
## 用例开发

#### 系统要求

已安装 Python 3.10 或更高版本。
你必须使用 Python MCP SDK 1.2.0 或更高版本。


```shell
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## server端

#### 实现部分

https://mcp-docs.cn/quickstart/server

server端对tools的实现-（具身这个场景下无论是client实现tool还是servers实现tool都无所谓，除非多具身-远程控制）

#### 执行流程

```shell

# 激活server环境
skills/demo_weather$ source .venv/bin/activate
# 将demo weather导入环境变量
skills/demo_weather$ export PYTHONPATH=$PYTHONPATH:../../capability/demo_weather
# 执行测试
skills/demo_weather$ uv run main.py
# 测试运行server
skills/demo_weather$ uv run weather.py

```
等任务来了处理

## client端

#### 实现部分

https://mcp-docs.cn/quickstart/client

实现connect_to_server(),并获取tool list

实现process_query(),通过api-keys调用LLM去使用tools

实现chat_loop(),用户交互界面

> 这里的执行是跑去server端执行去了-line 85
> 然后根据返回值再调大模型（这个逻辑可以改成执行一个序列/等报错再返回


连接方式分为两种sse和stdio - https://zhuanlan.zhihu.com/p/21166726702 / https://zhuanlan.zhihu.com/p/22449447146

```python
@mcp.tool()
async def get_alerts(state: str) -> str:
    """获取美国州的天气警报。

    Args:
        state: 两个字母的美国州代码（例如 CA、NY）
    """
```

注意这个注释部分，mcp tool从这里获得描述

#### 执行流程

MCP 服务器的进程由 MCP 客户端程序负责启动以下是测试部分

```shell

brain/mcp-client $ uv add mcp elasticsearch openai anthropic python-dotenv

# 终端路径
brain/mcp-client $ export PYTHONPATH=$PYTHONPATH:../../skills/demo_weather
brain/mcp-client $ export PYTHONPATH=$PYTHONPATH:../../capability/demo_weather

# 激活虚拟环境
brain/mcp-client $ source .venv/bin/activate

# 运行client
brain/mcp-client $ uv run client.py ../../skills/demo_weather/weather.py

```

此demo用deepseek支持tools的版本

```python
# shell运行client
|
## 创建MCP客户端
client = MCPClient()
| 
## 创建Client与server连接
await client.connect_to_server(sys.argv[1])|
|
## 用户输入问题
query = input("\nQuery: ").strip()
|
## 调用处理问题
response = await self.process_query(query)
response = self.anthropic.messages.create()
|
## client解析问题并扔给server
result = await self.session.call_tool(tool_name, tool_args)
|
## 再调大模型反馈-line100
response = self.anthropic.messages.create()
|
## 等处理完没有tools或者调用啥的
## ...
|
## 结束处理继续对话

```


## 总结&问题&想法

总结：
MCP框架基本上就是前后分开，LLM由client处理，server端仅处理functools的实现

优势在于其他client（支持client的平台等）也能通过server调用func tools，意思是说工具开发者可以通过Claude Desktop等工具开发tools

调试发现用deepseek情况下
给llm的tools的构建各个平台不同意，并非统一的tools
client解析llm规划向server请求func tools的流程还是得自己写
（XD根本做不到统一）

问题：
tools和tool execution需要在一起否则client没法打开server？应该是路径包含问题

想法：
1. 总分client-server
一个总的client-server管其他server的启动与否，启动后更新tools

2. capability返回bool或者int;  skill加封装返回字符串-skill中在失败时自动调用相关检查程序;
llm理解规划都是基于字符串做的，但下层的函数返回东西很简单

3. 不支持tools参数的大模型需要通过promote提示词来让llm去规划（promote也不支持我也没辙），然后改llm解析逻辑去调用server tools

4. 如果mcp不支持某个llm的格式可能得该改self.anthropic.messages.create的逻辑？