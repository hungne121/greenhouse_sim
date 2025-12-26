import speech_recognition as sr
from gtts import gTTS
import os
import time
import pygame
import pickle

class RobotGuide:
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.load_model()
        pygame.mixer.init()
        self.plants = {
            "hoa hồng": "Hoa hồng là loại cây bụi hoặc cây leo, nổi tiếng với vẻ đẹp và hương thơm quyến rũ.",
            "xương rồng": "Xương rồng là loài thực vật mọng nước, thường sống ở những vùng khô hạn.",
            "lan": "Hoa lan là một trong những họ thực vật có hoa lớn nhất, với nhiều màu sắc và hình dáng đa dạng."
        }

    def load_model(self):
        model_path = os.path.join(os.path.dirname(__file__), "model.pkl")
        try:
            with open(model_path, "rb") as f:
                self.classifier = pickle.load(f)
            print("Đã tải mô hình nhận diện ý định.")
        except FileNotFoundError:
            print("Không tìm thấy mô hình. Vui lòng chạy train_model.py trước.")
            self.classifier = None

    def speak(self, text):
        print(f"Robot: {text}")
        try:
            tts = gTTS(text=text, lang='vi')
            filename = "voice.mp3"
            tts.save(filename)
            
            # Sử dụng pygame để phát âm thanh
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
            
            # Unload để có thể xóa file
            pygame.mixer.music.unload()
            os.remove(filename)
        except Exception as e:
            print(f"Lỗi khi phát âm thanh: {e}")

    def listen(self):
        with sr.Microphone() as source:
            print("Đang lắng nghe...")
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                audio = self.recognizer.listen(source, timeout=5)
                text = self.recognizer.recognize_google(audio, language="vi-VN")
                print(f"Bạn: {text}")
                return text.lower()
            except sr.WaitTimeoutError:
                print("Không nghe thấy gì.")
                return ""
            except sr.UnknownValueError:
                print("Không hiểu bạn nói gì.")
                return ""
            except sr.RequestError:
                print("Lỗi kết nối dịch vụ nhận dạng giọng nói.")
                return ""

    def start_interaction(self):
        self.speak("Chào bạn, tôi là robot hướng dẫn. Bạn muốn thăm quan khu vực nào?")
        
        while True:
            response = self.listen()
            if not response:
                continue # Tiếp tục lắng nghe nếu không nghe thấy gì

            if self.classifier:
                try:
                    # Dự đoán xác suất
                    proba = self.classifier.predict_proba([response])[0]
                    max_proba = max(proba)
                    intent = self.classifier.classes_[proba.argmax()]
                    
                    print(f"Intent detected: {intent}, Confidence: {max_proba}")

                    if max_proba < 0.5: # Ngưỡng tin cậy
                        self.speak("Tôi không chắc chắn. Bạn có thể nói lại rõ hơn không?")
                    
                    elif intent == "whole":
                        self.speak("Tuyệt vời! Hãy đi theo tôi, chúng ta sẽ tham quan toàn bộ nhà kính.")
                        # Giả lập hành động dẫn đi
                        time.sleep(2)
                        self.speak("Đã tham quan xong. Bạn muốn đi đâu tiếp?")
                    
                    elif intent == "stop":
                        self.speak("Cảm ơn bạn đã thăm quan. Hẹn gặp lại!")
                        break
                    
                    elif intent == "specific_incomplete":
                        self.speak("Bạn muốn xem loại cây nào? Chúng tôi có hoa hồng, xương rồng và hoa lan.")
                    
                    elif intent == "specific_named":
                        # Logic suy luận: Tìm tên cây trong câu nói của người dùng
                        found_plant = None
                        for plant in self.plants.keys():
                            if plant in response:
                                found_plant = plant
                                break
                        
                        if found_plant:
                            self.speak(f"Được rồi, tôi sẽ dẫn bạn đến khu vực {found_plant}.")
                            time.sleep(2)
                            self.introduce_plant(found_plant)
                        else:
                            self.speak("Xin lỗi, hiện tại nhà kính chưa có loài cây này hoặc tôi chưa nghe rõ tên cây.")
                    
                    else:
                        self.speak("Tôi chưa hiểu ý bạn. Vui lòng nói rõ hơn.")
                except Exception as e:
                    print(f"Lỗi dự đoán: {e}")
                    self.speak("Có lỗi xảy ra khi xử lý yêu cầu của bạn.")
            else:
                self.speak("Lỗi hệ thống: Chưa tải được mô hình nhận diện.")
                break

    def introduce_plant(self, plant_name):
        plant_name = plant_name.lower()
        info = self.plants.get(plant_name)
        if info:
            self.speak(f"Đây là {plant_name}. {info}")
        else:
            self.speak(f"Xin lỗi, tôi chưa có thông tin về {plant_name}.")

if __name__ == "__main__":
    robot = RobotGuide()
    # Uncomment dòng dưới để test hội thoại
    robot.start_interaction()
    
    
