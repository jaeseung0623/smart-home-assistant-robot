import math
import requests
from datetime import datetime, timedelta

import asyncio
import edge_tts
import os
import time
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv("API_KEY")

class WeatherForecast:
    def __init__(self, api_key, use_tts=True):
        self.api_key = api_key
        self.use_tts = use_tts

    # 위도/경도 → 격자 변환
    def convert_to_grid(self, lat, lon):
        RE, GRID = 6371.00877, 5.0
        SLAT1, SLAT2 = 30.0, 60.0
        OLON, OLAT = 126.0, 38.0
        XO, YO = 43, 136

        DEGRAD = math.pi / 180.0
        re = RE / GRID
        slat1 = SLAT1 * DEGRAD
        slat2 = SLAT2 * DEGRAD
        olon = OLON * DEGRAD
        olat = OLAT * DEGRAD

        sn = math.tan(math.pi * 0.25 + slat2 * 0.5) / math.tan(math.pi * 0.25 + slat1 * 0.5)
        sn = math.log(math.cos(slat1) / math.cos(slat2)) / math.log(sn)
        sf = math.tan(math.pi * 0.25 + slat1 * 0.5)
        sf = (sf ** sn * math.cos(slat1)) / sn
        ro = math.tan(math.pi * 0.25 + olat * 0.5)
        ro = re * sf / (ro ** sn)

        ra = math.tan(math.pi * 0.25 + lat * DEGRAD * 0.5)
        ra = re * sf / (ra ** sn)
        theta = lon * DEGRAD - olon

        if theta > math.pi:
            theta -= 2.0 * math.pi
        if theta < -math.pi:
            theta += 2.0 * math.pi

        theta *= sn
        x = ra * math.sin(theta) + XO
        y = ro - ra * math.cos(theta) + YO

        return int(x + 1.5), int(y + 1.5)

    # base_date, base_time 계산
    def get_base_datetime(self):
        now = datetime.now()
        forecast_times = [(2,10),(5,10),(8,10),(11,10),(14,10),(17,10),(20,10),(23,10)]

        base_date = now.strftime("%Y%m%d")
        for hour, minute in reversed(forecast_times):
            forecast_time = now.replace(hour=hour, minute=minute, second=0, microsecond=0)
            if now >= forecast_time:
                base_time = f"{hour:02}00"
                return base_date, base_time

        base_date = (now - timedelta(days=1)).strftime("%Y%m%d")
        return base_date, "2300"

    # 기상 요약 문장 생성
    def get_weather_summary(self, lat, lon):
        base_date, base_time = self.get_base_datetime()
        x, y = self.convert_to_grid(lat, lon)
        fcst_date = datetime.now().strftime("%Y%m%d")
        current_time_hhmm = datetime.now().strftime("%H%M")

        url = "http://apis.data.go.kr/1360000/VilageFcstInfoService_2.0/getVilageFcst"
        params = {
            'serviceKey': self.api_key,
            'numOfRows': '1000',
            'pageNo': '1',
            'dataType': 'JSON',
            'base_date': base_date,
            'base_time': base_time,
            'nx': x,
            'ny': y
        }

        try:
            response = requests.get(url, params=params)
            response.raise_for_status()
            data = response.json()
            items = data.get('response', {}).get('body', {}).get('items', {}).get('item', [])

            if not items:
                return "⚠️ API 응답에 데이터가 없습니다."

            pty_map = {'0': '없음', '1': '비', '2': '비/눈', '3': '눈', '4': '소나기'}
            rain_times = []
            temps = []

            for item in items:
                if item['fcstDate'] == fcst_date and item['fcstTime'] > current_time_hhmm:
                    if item['category'] == 'PTY' and item['fcstValue'] != '0':
                        rain_times.append((item['fcstTime'], pty_map.get(item['fcstValue'], '알 수 없음')))
                    if item['category'] == 'TMP':
                        try:
                            temps.append(float(item['fcstValue']))
                        except ValueError:
                            pass

            if not temps:
                return "⚠️ 예보 시각 이후의 기온 정보가 없습니다."

            max_temp = max(temps)
            min_temp = min(temps)
            now = time.strftime("%H:%M")

            if rain_times:
                sorted_rain_times = sorted(rain_times)
                first_rain_time, first_pty = sorted_rain_times[0]
                last_rain_time, last_pty = sorted_rain_times[-1]
                if first_rain_time == last_rain_time:
                    rain_summary = f"{first_rain_time[:2]}시({first_pty})"
                else:
                    rain_summary = f"{first_rain_time[:2]}시({first_pty}), {last_rain_time[:2]}시({last_pty})"
                sentence = (
                    f"현재시간은 {now}, "
                    f"오늘 이후 예보 중 최고 기온은 {max_temp:.1f}도, "
                    f"최저 기온은 {min_temp:.1f}도이며, "
                    f"강수는 {rain_summary}에 예상됩니다."
                )
                if self.use_tts:
                    asyncio.run(tts_play(sentence))
            else:
                sentence = (
                    f"현재시간은 {now}, "
                    f"오늘 이후 예보 중 최고 기온은 {max_temp:.1f}도, "
                    f"최저 기온은 {min_temp:.1f}도이며, "
                    f"강수는 없습니다."
                )
                if self.use_tts:
                    asyncio.run(tts_play(sentence))

            return sentence

        except requests.exceptions.RequestException as e:
            return f"API 호출 오류: {e}"
        except ValueError as e:
            return f"JSON 파싱 오류: {e}"

# edge-tts로 mp3 저장하고 재생
async def tts_play(text: str):
    communicate = edge_tts.Communicate(text=text, voice="ko-KR-SunHiNeural")
    await communicate.save("/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/weathertts.mp3")
    os.system("mpg123 /home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/weathertts.mp3")

# 테스트 실행
if __name__ == "__main__":
    API_KEY = api_key
    lat, lon = 37.495, 126.856  # 서울 구로구 기준
    wf = WeatherForecast(API_KEY)
    summary = wf.get_weather_summary(lat, lon)
    print(summary)
