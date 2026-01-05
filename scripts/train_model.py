#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pickle
import itertools
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.naive_bayes import MultinomialNB
from sklearn.pipeline import make_pipeline
import os

def generate_permutations(templates, slots):
    """
    Generates sentences by filling templates with all combinations of slots.
    Example: 
      Template: "{verb} {noun}"
      Slots: verb=[eat, see], noun=[apple, pear]
      Result: [eat apple, eat pear, see apple, see pear]
    """
    results = []
    for tmpl in templates:
        # Check which keys are in this template
        keys = [k for k in slots.keys() if "{" + k + "}" in tmpl]
        if not keys:
            results.append(tmpl)
            continue
            
        # Create all combinations for these keys
        value_lists = [slots[k] for k in keys]
        for combination in itertools.product(*value_lists):
            mapping = dict(zip(keys, combination))
            results.append(tmpl.format(**mapping))
    return list(set(results)) # Deduplicate

def train_and_save_model():
    data = []
    labels = []

    def add_list(phrase_list, label):
        for p in phrase_list:
            data.append(p)
            labels.append(label)

    # ================= COMMON SLOTS =================
    common_slots = {
        "robot": ["robot", "bạn", "mày", "em", "cậu", "trợ lý"],
        "please": ["làm ơn", "nhé", "giúp tôi", "đi nào", "hộ cái", "với"],
        "verb_go": ["đi", "dẫn", "đưa", "di chuyển", "chạy", "hướng dẫn"],
        "target_loc": ["đến chỗ", "tới vị trí", "qua khu", "lại chỗ", "sang bên"],
        "noun_plant": ["cây", "hoa", "bông"],
        "verb_see": ["xem", "ngắm", "nhìn", "quan sát", "tìm hiểu"],
        "question_what": ["gì", "nào", "ra sao", "thế nào", "như nào"],
        "time_now": ["ngay", "luôn", "bây giờ", "lập tức"]
    }

    # ================= INTENT GENERATION =================

    # --- 1. WHOLE TOUR (Tham quan toàn bộ) ---
    # Target: >150 samples
    tour_templates = [
        "{verb_go} {scope} {please}",
        "{verb_go} {scope}",
        "muốn {verb_see} {scope}",
        "bắt đầu {noun_tour}",
        "{noun_tour} {scope}",
        "dẫn đường {scope}"
    ]
    tour_slots = {
        **common_slots,
        "scope": ["hết", "toàn bộ", "tất cả", "một vòng", "quanh vườn", "tổng thể", "mọi thứ", "hết các cây"],
        "noun_tour": ["tour", "chuyến tham quan", "hành trình", "buổi đi dạo"],
    }
    add_list(generate_permutations(tour_templates, tour_slots), "whole_tour")

    # --- 1b. CONTINUE TOUR (Tiếp tục tour) ---
    continue_templates = [
        "{continue_verb} tour",
        "{continue_verb} {verb_go}",
        "{continue_verb} tham quan",
        "cây {next_word}",
        "{verb_go} cây khác",
        "{verb_go} {next_word}"
    ]
    continue_slots = {
        **common_slots,
        "continue_verb": ["tiếp tục", "tiếp", "đi tiếp"],
        "next_word": ["tiếp theo", "kế tiếp", "sau", "khác", "nữa"]
    }
    add_list(generate_permutations(continue_templates, continue_slots), "continue_tour")
    
    # --- 1c. LIST PLANTS (Liệt kê cây) ---
    list_templates = [
        "ở đây có {what} cây",
        "{what} loại cây",
        "nhà kính trồng {what}",
        "danh sách {noun_plant}",
        "kể tên các {noun_plant}",
        "có bao nhiêu loại cây"
    ]
    list_slots = {
        **common_slots,
        "what": ["những", "các", "mấy", "bao nhiêu", "gì"]
    }
    add_list(generate_permutations(list_templates, list_slots), "list_plants")
    print(f"Training samples generated: {len(data)} across {len(set(labels))} intents.")


    # --- 2. NAVIGATION (Đến vị trí cụ thể) ---
    # Target: >200 samples
    nav_templates = [
        "{verb_go} {target_loc} {noun_plant}",
        "muốn {verb_see} {noun_plant}",
        "{noun_plant} trồng ở đâu",
        "dẫn {please} {target_loc} {noun_plant}",
        "tìm {noun_plant} giúp {please}",
        "vị trí của {noun_plant}",
        "{verb_go} tìm {noun_plant}"
    ]
    # Note: Specific plant names are handled by generic NLU + Entity Extraction in Controller
    # But we train with generic words to capture structure
    nav_slots = {
        **common_slots,
        "noun_plant": ["cây", "hoa", "bụi cây", "chậu hoa", "cúc", "hồng", "lan", "sen đá", "ly", "hoa lan"] 
    }
    add_list(generate_permutations(nav_templates, nav_slots), "navigation_request")


    # --- 3. STOP ACTION (Pause) ---
    # Target: >100 samples
    stop_templates = [
        "{verb_stop} {please}",
        "{verb_stop} {time_now}",
        "{verb_wait} {please}",
        "{verb_wait} một chút",
        "{robot} {verb_stop}",
        "khoan đã", "từ từ thôi"
    ]
    stop_slots = {
        **common_slots,
        "verb_stop": ["dừng", "dừng lại", "đứng lại", "ngưng", "stop", "đứng yên"],
        "verb_wait": ["đợi", "chờ", "khoan"]
    }
    add_list(generate_permutations(stop_templates, stop_slots), "stop_action")


    # --- 3b. QUIT SYSTEM (Exit) ---
    # Target: >100 samples
    quit_templates = [
        "{verb_quit} {please}",
        "{verb_quit} chương trình",
        "{verb_bye} {robot}",
        "không dùng nữa", "nghỉ thôi", "kết thúc {please}"
    ]
    quit_slots = {
        **common_slots,
        "verb_quit": ["tắt", "tắt máy", "thoát", "shutdown", "ngắt kết nối", "dừng hoạt động"],
        "verb_bye": ["tạm biệt", "bye", "goodbye", "chào"]
    }
    add_list(generate_permutations(quit_templates, quit_slots), "quit_system")


    # --- 4. ASK VARIETY (Giống loài) ---
    # Target: >100 samples
    variety_templates = [
        "đây là {noun_type} {question_what}",
        "{noun_plant} này là {noun_type} {question_what}",
        "{noun_origin} ở đâu",
        "{noun_origin} của {noun_plant}",
        "có những {noun_type} nào",
        "tên khoa học là {question_what}"
    ]
    variety_slots = {
        **common_slots,
        "noun_type": ["giống", "loài", "loại", "tên"],
        "noun_origin": ["xuất xứ", "nguồn gốc", "quê quán", "xuất sứ"]
    }
    add_list(generate_permutations(variety_templates, variety_slots), "ask_variety")


    # --- 5. ASK TECHNIQUE (Kỹ thuật) ---
    # Target: >150 samples
    tech_templates = [
        "{verb_tech} {noun_plant} {question_what}",
        "cách {verb_tech} {noun_plant}",
        "hướng dẫn {verb_tech}",
        "dùng {noun_material} {question_what}",
        "kỹ thuật {verb_tech} là {question_what}",
        "bao lâu thì {noun_result}"
    ]
    tech_slots = {
        **common_slots,
        "verb_tech": ["trồng", "nhân giống", "gieo hạt", "giâm cành", "bón phân", "cắt tỉa"],
        "noun_material": ["đất", "giá thể", "phân bón", "chậu"],
        "noun_result": ["thu hoạch", "ra hoa", "lớn", "có quả"]
    }
    add_list(generate_permutations(tech_templates, tech_slots), "ask_technique")


    # --- 6. ASK CARE (Chăm sóc) ---
    # Target: >150 samples
    care_templates = [
        "{verb_care} {noun_plant} {question_what}",
        "{question_how} để {verb_care}",
        "có cần {verb_action} không",
        "{noun_factor} {question_what}",
        "bị {noun_problem} thì sao"
    ]
    care_slots = {
        **common_slots,
        "verb_care": ["chăm sóc", "tưới nước", "nuôi dưỡng", "bảo vệ"],
        "verb_action": ["tưới nhiều", "tưới ít", "phơi nắng", "che nắng", "phun thuốc"],
        "noun_factor": ["ánh sáng", "nhiệt độ", "độ ẩm", "lượng nước"],
        "noun_problem": ["sâu", "bệnh", "nấm", "rệp", "vàng lá", "héo"],
        "question_how": ["làm sao", "như thế nào", "cách nào"]
    }
    add_list(generate_permutations(care_templates, care_slots), "ask_care")
    
    
     # --- 7. GREETING_SMALLTALK (Chào hỏi) ---
    greet_templates = [
        "{hello} {robot}",
        "{hello} {please}",
        "{hello} bạn",
        "bạn tên {question_what}",
        "giới thiệu bản thân {please}"
    ]
    greet_slots = {
        **common_slots,
        "hello": ["xin chào", "chào", "hi", "hello", "alo", "ê"]
    }
    add_list(generate_permutations(greet_templates, greet_slots), "greeting")


    # --- 8. PLANT STORY (Sự tích) ---
    story_templates = [
        "{verb_tell} {noun_story} {noun_plant}",
        "{noun_plant} có {noun_story} {question_what}",
        "{noun_meaning} của {noun_plant}",
        "tại sao gọi là {noun_plant}"
    ]
    story_slots = {
        **common_slots,
        "verb_tell": ["kể", "nói", "cho nghe"],
        "noun_story": ["chuyện", "sự tích", "truyền thuyết", "lịch sử", "câu chuyện"],
        "noun_meaning": ["ý nghĩa", "biểu tượng", "phong thủy"]
    }
    add_list(generate_permutations(story_templates, story_slots), "plant_story")


    # --- 9. USER COMPLAINT ---
    complaint_templates = [
        "{robot} {verb_wrong}",
        "thông tin {verb_wrong}",
        "{verb_wrong} rồi",
        "không phải {noun_plant} đâu",
        "nói {adverb_bad} quá"
    ]
    complaint_slots = {
        **common_slots,
        "verb_wrong": ["sai", "nhầm", "lỗi", "bậy", "không đúng"],
        "adverb_bad": ["chán", "dở", "linh tinh", "sai bét"]
    }
    add_list(generate_permutations(complaint_templates, complaint_slots), "user_complaint")
    
    
    # --- 10. USER PRAISE ---
    praise_templates = [
        "{robot} {adj_good} {exclaim}",
        "{adj_good} quá",
        "làm {adj_good} {exclaim}",
        "cảm ơn {robot}",
        "rất {adj_good}"
    ]
    praise_slots = {
        **common_slots,
        "adj_good": ["giỏi", "hay", "tốt", "thông minh", "tuyệt vời", "xuất sắc", "đáng yêu"],
        "exclaim": ["lắm", "quá", "ghê", "nha", "thật"]
    }
    add_list(generate_permutations(praise_templates, praise_slots), "user_praise")
    

    print(f"AUTOMATIC GENERATION COMPLETE.")
    print(f"Total training samples: {len(data)}")
    print(f"Total intents: {len(set(labels))}")
    print("---------------------------------------------------")
    
    model = make_pipeline(CountVectorizer(ngram_range=(1, 3)), MultinomialNB())
    model.fit(data, labels)
    
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.pkl")
    with open(save_path, "wb") as f:
        pickle.dump(model, f)
    print("Model saved to", save_path)

if __name__ == "__main__":
    train_and_save_model()
