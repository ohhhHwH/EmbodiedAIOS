

from openai import OpenAI



def send_messages(client, messages, tools=None):
    response = client.chat.completions.create(
        model="deepseek-chat",
        messages=messages,
        tools=tools
    )
    return response.choices[0].message

# 初始化OpenAI客户端
def initialize_client(api_key, base_url):
    # 使用DeepSeek的API密钥和基础URL初始化OpenAI客户端
    client = OpenAI(
        api_key=api_key,
        base_url=base_url,
    )
    return client

# 移动到指定位置
def go_to_location(location):
    # 调用移动模型的移动函数
    print(f"Going to {location}")
    # 返回到达位置的结果
    return f"at {location} now"

# 抓取物品
def grab_object(object):
    # 调用抓取模型的抓取函数
    print(f"Grabbing {object}")
    # 返回抓取结果
    return f"{object} grabbed"

# 返回物品的位置
def identify_object(object):
    # 调用抓取模型的抓取函数
    print(f"Identifying {object}")
    # 返回抓取结果
    return f"{object} at location x, y, z" 

def func_call(messages, tool_calls):
    for tool_call in tool_calls:
        if tool_call.type == "function":
            function = tool_call.function
            funcName = (str)(function.name)
            funcArguments = function.arguments
            funcArguments = eval(funcArguments)

            if funcName == "go_to_location":
                location = funcArguments.get("location")
                content = go_to_location(location)

            elif funcName == "grab_object":
                object = funcArguments.get("object")
                content = grab_object(object)
            elif funcName == "identify_object":
                object = funcArguments.get("object")
                content = identify_object(object)
            else:
                content = "Unknown function call"
                print(f"Unknown function call: {funcName}")

        # 输出函数调用的结果 
        print(f"Function call result: {content}")
        messages.append(
            {"role": "tool", "tool_call_id": tool_call.id, "content": content})


def main(args=None):
    
    # 根据args获取初始化变量
    # 默认 静默提示词
    system_prompt = """
    You are a Robot control center, you can control the robot to do some tasks.
    The robot can do the following things:

    - go_to_location(location: str)
    - grab_object(object: str)
    - identify_object(object: str)

    You can call these functions with the parameters you need.

    """

    # 可供调用的函数列表
    tools = [
        {# 移动到指定位置
            "type": "function",
            "function": {
                "name": "go_to_location",
                "description": "Go to a location",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "The location to go to",
                        }
                    },
                    "required": ["location"]
                },
            }
        },
        {# 抓取物体
            "type": "function",
            "function": {
                "name": "grab_object",
                "description": "Grab an object",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object": {
                            "type": "string",
                            "description": "The object to grab",
                        }
                    },
                    "required": ["object"]
                },
            }
        },
        {# 找到物体
            "type": "function",
            "function": {
                "name": "identify_object",
                "description": "Find an object",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object": {
                            "type": "string",
                            "description": "The object to find",
                        }
                    },
                    "required": ["object"]
                },
            }
        }
    ]
    
    api_key = "sk-"
    base_url = "https://api.deepseek.com"


    # 初始化OpenAI客户端
    client = initialize_client(api_key, base_url)

    question = "find and grab the red ball on the table then hold the ball go to the kitchen"
    # question = "what the weather in beijing"

    # 1.相关问题/可以解决的问题-返回调用函数信息
    # messages = [{"role": "system", "content": system_prompt} , {"role": "user", "content": question}]
    messages = [{"role": "user", "content": question}]


    message = send_messages(client, messages, tools)
    # 如果conetent没有内容不打印,打印tool_calls的size
    while True:
        if message.content != "":
            print(f"Model>\t {message.content}")
            break
        else:
            
            print(f"Model>\t using function call")
            # 2.调用工具
            messages.append(message)
            func_call(messages, message.tool_calls)
            # 3.将调用工具的结果返回给模型进行回答
            message = send_messages(client, messages, tools)
    
    # 完成一次任务
    print("finish")




'''

Model>	 ChatCompletionMessage
(content='', refusal=None, 
role='assistant', audio=None, 
function_call=None, 
tool_calls=[ChatCompletionMessageToolCall(id='call_0_b7a79846-2b61-47e8-8169-6d9e7cf7ccfa', 
function=Function(arguments='{"object": "red ball"}', name='grab_object'),
    type='function', index=0), 
ChatCompletionMessageToolCall(id='call_1_aa97535e-65d2-478d-816b-50b59a7f693d', 
function=Function(arguments='{"location": "kitchen"}', name='go_to_location'), 
type='function', index=1)])


'''
