plate_positions = {
    "접시": [[6],[134.6, 141.9, 293.5, 176.63, 57.01, -158.11]]
}

bread_positions = {
    "아래 빵": [[7],[140.3, -64.6, 363.4, 90.63, 46.55, 92.48]],
    "위 빵": [[1],[132.6, -167.8, 286.3, 99.72, 49.66, 92.16]]
}

meat_positions = {
    "불고기 샌드위치": [[4],[129.6, 99.1, 339.9, 94.91, 57.83, 120.37]],
    "새우 샌드위치": [[3],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
    "베이컨 샌드위치": [[2],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
}

sauce_positions = {
    "이탈리안": [[5],[134.6, 141.9, 293.5, 176.63, 57.01, -158.11]],
    "칠리": [[6],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
}

vegetable_positions = {
    "양상추": [[3],[140.1, 22.7, 266.5, 118.97, 57.86, 133.68]],
    "로메인": [[8],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
    "바질": [[9],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
}

cheese_positions = {
    "슬라이스 치즈": [[10],[164.5, -126.1, 267.3, 100.36, 51.26, 100.07]],
    "슈레드 치즈": [[15],[164.5, -126.1, 267.3, 100.36, 51.26, 100.07]],
    "모짜렐라 치즈": [[12],[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
}

# check_id = {plate_positions,bread_positions,meat_positions,sauce_positions,vegetable_positions,cheese_positions}

def get_plate_position(plate_name):
    return plate_positions.get(plate_name, None)

def get_bread_position(bread_name):
    return bread_positions.get(bread_name, None)

def get_meat_position(meat_name):
    return meat_positions.get(meat_name, None)

def get_sauce_position(sauce_name):
    return sauce_positions.get(sauce_name, None)

def get_vegetable_position(vegetable_name):
    return vegetable_positions.get(vegetable_name, None)

def get_cheese_position(cheese_name):
    return cheese_positions.get(cheese_name, None)
