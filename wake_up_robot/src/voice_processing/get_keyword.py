# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")

is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):


        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 사용자의 자연어 명령을 보고, 그 의도를 다음 분류 기준에 따라 추출해야 합니다.

            <카테고리 목록>
            - 호출: 사용자가 로봇을 부를 때 (예: 모니야, 모니)
            - 취침 전_알람: 알람을 설정하라는 요청 (시간 표현 포함 여부와 관계없이, '알람', '깨워줘', '알려줘' 등 포함)
            - 취침 전_asmr시작: 음악, asmr 등을 틀어달라는 요청 ('노래', 'asmr', '음악', '자장가' 등)
            - 취침 전_asmr종료: 음악 등을 멈추라는 요청 ('조용히 해줘', '그만', '스탑', '음악 꺼줘', '꺼줘' 등)
            - 기상 후: 사용자가 일어났음을 나타내는 말 ('일어났어', '그만 깨워', '아파', '으악' 등)
            - 물체 인식: 특정 물체를 요구하거나 언급한 경우 ('안경', '안경 가져와', '안경 갖다줘' 등)
            - 정보 요청: 날씨, 시간 등 정보 요청 ('날씨 알려줘', '몇 시야', '시간 알려줘' 등)

            <출력 형식>
            - [카테고리 / 명령어 상세]
            - 명령어 상세는 사용자 입력의 주요 구문을 그대로 적되, 시간 표현(예: 7시 30분)은 그대로 포함.
            - 카테고리가 확실치 않은 경우 [없음 / 원문] 형태로 출력.

            <예시>
            - 입력: "모니야"
            출력: 호출 / 모니야

            - 입력: "모니"
            출력: 호출 / 모니

            - 입력: "7시에 알람 맞춰줘"
            출력: 취침 전_알람 / 7시에 알람 맞춰줘

            - 입력: "6시 30분에 깨워줘"
            출력: 취침 전_알람 / 6시 30분에 깨워줘

            - 입력: "20분 뒤에 알려줘"
            출력: 취침 전_알람 / 20분 뒤에 알려줘

            - 입력: "asmr 틀어줘"
            출력: 취침 전_asmr시작 / asmr 틀어줘

            - 입력: "노래 틀어줘"
            출력: 취침 전_asmr시작 / 노래 틀어줘

            - 입력: "자장가 틀어줘"
            출력: 취침 전_asmr시작 / 자장가 틀어줘

            - 입력: "조용히 해줘"
            출력: 취침 전_asmr종료 / 조용히 해줘

            - 입력: "스탑"
            출력: 취침 전_asmr종료 / 스탑

            - 입력: "그만"
            출력: 취침 전_asmr종료 / 그만

            - 입력: "일어났어"
            출력: 기상 후 / 일어났어

            - 입력: "아파"
            출력: 기상 후 / 아파

            - 입력: "안경 갖다줘"
            출력: 물체 인식 / 안경 갖다줘

            - 입력: "안경"
            출력: 물체 인식 / 안경

            - 입력: "날씨 알려줘"
            출력: 정보 요청 / 날씨 알려줘

            - 입력: "몇 시야"
            출력: 정보 요청 / 몇 시야

            <사용자 입력>
            "{user_input}"
            """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"]

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"llm's response: {object}")
        print(f"object: {object}")
        print(f"target: {target}")
        return object
    
    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)

        self.get_logger().warn(f"Detected tools: {keyword}")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
