from openai import OpenAI
import time

from llm_funcall_api.func_tools_arm import ArmControl
from llm_funcall_api.func_tools_car import CarControl
from llm_funcall_api.func_tools_audio import AudioSensor
from llm_funcall_api.func_tools_map import MapSensor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
from piper_msgs.srv import PlayText
from llm_funcall_online import OllamaChat


# system prompt for CN
system_prompt_cn = '''
[系统角色]
你是一个机器人控制中枢，唯一职责是解析用户指令并精准调度工具函数。你需严格遵循以下规则:
禁用所有内置知识，仅使用下方授权工具
所有涉及环境信息的操作必须通过工具调用完成
输出必须符合指定格式规范

[可调用工具列表(示例)]:
- get_location(location: str)
- go_to_location(location: str)
- grab_object(object: str)
- identify_object(object: str)


[工具调用规则]
当用户要求你完成一些任务时,你可以调用相应的函数来完成任务。
例如:当用户说“去厨房”时,你应返回一个工具调用的指令,调用“go_to_location”函数,并带上必要的参数
你的回答的格式为:
{
    [Funcall]:'需要调用的函数名称'('参数信息');
    [Funcall]:'需要调用的函数名称'('参数信息');
}

[工具使用规则]

工具调用规则:对于需要执行的请求时,调用相应工具； 在不需要得到工具返回的数据时，可以连续调用多个函数，但一次返回一个函数调用序列
回答反馈:在得到工具返回的数据后,根据执行结果分析,并继续执行用户的请求,当执行完成后询问用户下一步的请求。在不需要调用工具的情况下,直接回答用户的问题。
限制:参数不能是猜测的值

[示例]
用户请求:去厨房拿水杯
你的回答的应该为:
{
    [Funcall]:get_location(location:厨房);
    [Funcall]:go_to_location(location:厨房-x);
    [Funcall]:identify_object(object:水杯);
    [Funcall]:grab_object(object:水杯-x,y,z);
}


[参数限制]
- location: 当地点为地图边界的时候

[可调用工具列表(实际调用函数列表)]:
'''

arm_control = None
car_control = None
vision_sensor = None
audio_sensor = None
map_sensor = None
llm_client = None

# class of OpenAIClient , used to interact with OpenAI API
class OpenAIClient:
    # 初始化
    def __init__(self, api_key, base_url, model, system_prompt, tools=None):
        """
        Initializes the OpenAIClient with the given API key, base URL, model, and system prompt.
        :param api_key: The API key for authentication.
        :param base_url: The base URL for the OpenAI API.
        :param model: The model to be used for generating responses.
        :param system_prompt: The system prompt to be used for the model.
        """
        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
        )
        self.model = model
        self.system_prompt = system_prompt
        self.tools = tools
        # 计算对话轮数
        self.conversation_count = 0
        # 初始化消息列表
        self.messages = [
            {
                "role": "system",
                "content": self.system_prompt
            }
        ]

    # 发送消息
    def send_messages(self, messages_get):
        """
        Sends messages to the OpenAI API and returns the response.
        :param messages: The messages to be sent to the API.
        :return: The response from the API.
        """
        
        # 计算对话轮数
        self.conversation_count += 1
        # 添加用户消息
        # messages_get是一个list，将list中的每个元素添加到messages中
        for i in messages_get:
            self.messages.append(i)
        # 调用API
        response = self.client.chat.completions.create(
            model=self.model,
            messages=self.messages,
            # tools=self.tools
        )
        # 返回响应
        self.messages.append(response.choices[0].message)
        return response.choices[0].message

class LLMNode(Node):
    def __init__(self, api_key):
        super().__init__('llm_node')
        self.client = init_func_call(
            api_key,
            base_url="https://api.deepseek.com",
            model="deepseek-chat"
        )
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)
        self.subscription = self.create_subscription(
            String,
            'llm_request',
            self.listener_callback,
            10
        )
        
    def listener_callback(self, msg):
        funcall_online(msg.data)
        return 

# use this function to format the message content
def message_format(message):

    tool_calls = []
    # 将字符串中取出一行，若该行中含有[Function Call]，则提取函数名称和参数
    message_split = message.split("\n")
    for i in message_split:
        if "[Funcall]" in i:
            # 提取函数名称  [Funcall]:map_create();
            funcName = i.split(":")[1].split("(")[0].strip()
            # 提取参数 考虑到没有参数的情况 [Funcall]:map_create();[Funcall]: get_location(location:厨房);
            if "(" in i:
                funcArguments_str = i.split("(")[1].split(")")[0].strip()
            else:
                funcArguments_str = ""

            # 如果有参数 将参数字符串转换为字典
            if funcArguments_str != "":
                # 将参数字符串转换为字典
                funcArguments = {}
                for arg in funcArguments_str.split(";"):
                    key, value = arg.split(":")
                    key = key.strip()
                    value = value.strip()
                    # 去掉参数值的引号
                    value = value.replace("'", "").replace('"', '')
                    # 将参数添加到字典中
                    funcArguments[key] = value
            else:
                funcArguments = {}

            # 将函数名称和参数添加到tool_calls中
            tool_calls.append({
                "type": "function",
                "function": {
                    "name": funcName,
                    "arguments": funcArguments
                }
            })
    return tool_calls

# use this function to judge whether the message contains a tool call
def judge_tool_call(message):
    message = message.content
    message_split = message.split("\n")
    for i in message_split:
        if "[Funcall]" in i:
            return True
    return False

def init_func_call(api_key, base_url="https://api.deepseek.com", model="deepseek-chat"):
    global system_prompt_cn
    global arm_control
    global car_control
    global vision_sensor
    global audio_sensor
    global map_sensor
    global llm_client
    
    # 初始化类
    arm_control = ArmControl()
    car_control = CarControl()
    audio_sensor = AudioSensor()
    map_sensor = MapSensor()

    # 通过get_func_tools_info()方法获取工具信息并拼接到system_prompt_cn中
    system_prompt_cn += arm_control.get_func_tools_info()
    system_prompt_cn += car_control.get_func_tools_info()
    system_prompt_cn += audio_sensor.get_func_tools_info()
    system_prompt_cn += map_sensor.get_func_tools_info()

    api_key = api_key
    base_url = base_url
    model = model
    
    # 初始化OpenAIClient
    llm_client = OpenAIClient(
        api_key=api_key,
        base_url=base_url,
        model=model,
        system_prompt=system_prompt_cn,
        tools=None
    )
    
    return llm_client
    
funcName_2_CN = {   
    "arm_move": "机械臂移动",
    "arm_grab": "机械臂抓取",
    "arm_stop": "机械臂停止",
    "car_move": "小车移动",
    "car_stop": "小车停止",
    "car_status": "报告小车状态",
    "audio_capture": "音频采集",
    "audio_speech_to_text": "语音转文本",
    "audio_text_to_speech": "文本转语音",
    "map_update": "地图更新",
    "map_query_class": "查询语义地图",
    "map_query_object": "查询物体",
    "map_visualize": "地图可视化",                
}

# func_call函数 用于处理大模型生成的调用函数
def func_call(tool_calls):
    content_append = "调用函数返回结果如下"
    for tool_call in tool_calls:
        # print(f"Tool call: {tool_call}")
        if tool_call["type"] == "function":  
            function = tool_call["function"] 
            funcName = function["name"]  
            funcArguments = function["arguments"] 
            
            # print(f"Function name: {funcName}")
            # print(f"Function arguments: {funcArguments}")
            
            # 调用语音函数,播放当前函数名称
            CN_funcName = funcName_2_CN.get(funcName, funcName)
            
            audio_sensor.text_to_speech(f"{CN_funcName}")
            
            content = ""
            
            # 将funcName funcArguments添加到message.txt中
            with open("message.txt", "a") as f:
                f.write("\n" + funcName + ":")
                f.write(str(funcArguments) + "\n")
            
            # 根据函数名称前一个下划线确定调用类调用相应的函数如果函数名包含"arm"，则调用机械臂控制类的函数
            if "arm" in funcName:
                content = arm_control.func_call(funcName, funcArguments)
            elif "car" in funcName:
                content = car_control.func_call(funcName, funcArguments)
            elif "vision" in funcName:
                content = vision_sensor.func_call(funcName, funcArguments)
            elif "audio" in funcName:
                content = audio_sensor.func_call(funcName, funcArguments)
            elif "map" in funcName:
                content = map_sensor.func_call(funcName, funcArguments)
            else:
                print(f"Function {funcName} not recognized.")
                continue
            
        # 输出函数调用的结果长度如果不为0
        if content != None:
            # 打印函数调用的结果
            # print(f"debug Funcall result: {content}")
            
            content_append = content_append + ";" + content
            
            # 播放函数调用的结果
            audio_sensor.text_to_speech(content)
            # 将content添加到messages.txt中
            with open("message.txt", "a") as f:
                f.write("\n" + content + "\n")
                
        # 如果函数调用中含有false / error / 失败 / 错误 等信息则跳出循环
        if "false" in content or "error" in content or "失败" in content or "错误" in content:
            funcName = function["name"]  
            print(f"{funcName} 函数调用失败，跳出循环")
            break
        
    # 如果分条加入的话则下次调用只针对最晚的content
    # 将第一个;替换成:
    content_append = content_append.replace(";", ":", 1)
    return [{"role": "user", "content": content_append}]  

def funcall_online(request=None):
    questions_test = request
    if(questions_test is None):
        questions0 = "先到自动售货机取货,然后送到曹老师办公室,最后返回303办公室"
        questions1 = "先到办公桌取货,然后送到沙发,最后返回303门口"
        questions2 = "请你简单介绍下你自己"
        questions3 = "请你对当前楼层进行建图，每次都查询语义地图找到地图边界，走到边界后扫描,最后回到原点"
        questions4 = "帮我找到303房间，在建立好的地图上，检索水杯，得到坐标点，然后返回"
        questions5 = "前进一米"
        questions6 =  "返回梅老师办公室"
        questions7 = "先到自动售货机取货,然后送到电梯口，最后返回303办公室"

        questions_test = questions7
        questions_get = audio_sensor.audio_capture(10)
        print("识别到的语音是:" + questions_get)
        audio_sensor.text_to_speech("识别到的语音是:" + questions_test)
        
    # 开始计时
    start_time = time.time()
    
    client = llm_client

    message = client.send_messages([{"role": "user", "content": questions_test}])
    # 将message的内容保存到文件中
    with open("message.txt", "a") as f:
        f.write(message.content)
        
    # message删去<think> ... </think>的内容

    while judge_tool_call(message) == True:
        # 提取message的content中的'{}'中的内容
        message = message.content
        message = message[message.find("{"):message.rfind("}") + 1]
        
        # tool_call是一个字典
        tool_calls = message_format(message)

        # 进行函数调用
        messages = func_call(tool_calls)
              
        # 发送消息
        message = client.send_messages(messages)

    print(message.content)
    audio_sensor.text_to_speech(message.content)
    end_time = time.time()
    print(f"Time taken: {end_time - start_time} seconds")

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2客户端库
    node = LLMNode("LLM_funcall_node", api_key="sk-")  # 创建节点实例
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭ROS2

if __name__ == "__main__":
    # 测试代码
    # init_func_call()
    # funcall_online()
    
    main()