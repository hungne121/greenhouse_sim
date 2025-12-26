#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import random

class KnowledgeManager:
    def __init__(self):
        self.plants_data = {}
        self.load_data()

    def load_data(self):
        """Load plant data from json file"""
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            json_path = os.path.join(current_dir, 'plants.json')
            
            with open(json_path, 'r', encoding='utf-8') as f:
                self.plants_data = json.load(f)
            print(f"[Knowledge] Loaded info for {len(self.plants_data)} plants.")
        except Exception as e:
            print(f"[Knowledge] Error loading plants.json: {e}")

    def get_all_plant_names(self):
        """Return list of available plant names"""
        return list(self.plants_data.keys())

    def get_plant_details(self, plant_name):
        """Get full intro string for a plant"""
        plant_name = plant_name.lower()
        data = self.plants_data.get(plant_name)
        if not data:
            return None
        
        return f"{plant_name} ({data['scientific_name']}). {data['description']} {data['characteristics']}"

    def get_care_info(self, plant_name):
        """Get care instruction string"""
        plant_name = plant_name.lower()
        data = self.plants_data.get(plant_name)
        if not data:
            return None
        
        return f"Về cách chăm sóc {plant_name}: Nước: {data['care_water']} Ánh sáng: {data['care_light']}"

    def get_fun_fact(self, plant_name):
        """Get a fun fact"""
        plant_name = plant_name.lower()
        data = self.plants_data.get(plant_name)
        if not data:
            return None
        return f"Bạn có biết? {data['fun_fact']}"

    def answer_question(self, text):
        """
        Simple keyword-based QA.
        Returns (Answer String, Confidence Enum/Score)
        """
        text = text.lower()
        
        # 1. Identify Plant
        found_plant = None
        for name in self.plants_data.keys():
            if name in text:
                found_plant = name
                break
        
        if not found_plant:
            return "Tôi xin lỗi, tôi chưa hiểu bạn đang hỏi về loài cây nào trong nhà kính.", False

        data = self.plants_data[found_plant]

        # 2. Identify Attribute
        if any(w in text for w in ["tưới", "nước", "ẩm", "chăm sóc"]):
            return f"Đối với {found_plant}: {data['care_water']}", True
        
        elif any(w in text for w in ["nắng", "sáng", "ánh sáng", "môi trường"]):
             return f"Về ánh sáng cho {found_plant}: {data['care_light']}", True
             
        elif any(w in text for w in ["nguồn gốc", "xuất xứ", "đến từ"]):
             return f"{found_plant} {data['origin']}", True
             
        elif any(w in text for w in ["đặc điểm", "là gì", "thế nào", "ra sao"]):
             return f"{found_plant}: {data['description']} {data['characteristics']}", True
             
        elif any(w in text for w in ["thú vị", "bí mật", "hay ho"]):
             return f"Có một điều thú vị là: {data['fun_fact']}", True

        # Default fallback if plant found but question unclear
        return f"Về {found_plant}, đây là loài cây: {data['description']}. Bạn muốn biết về cách tưới nước hay ánh sáng?", True

# Simple test
if __name__ == "__main__":
    km = KnowledgeManager()
    print(km.answer_question("tưới nước cho hoa hồng thế nào"))
    print(km.answer_question("xương rồng có nguồn gốc từ đâu"))
