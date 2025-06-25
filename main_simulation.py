#外部
import pathlib
import jupedsim as jps
import matplotlib.pyplot as plt
import numpy as np
from shapely import Polygon
from shapely.geometry import Point
from shapely.ops import unary_union
import math
from shapely.geometry import Point
from shapely.validation import explain_validity
import matplotlib
import random
import pedpy
from jupedsim.internal.notebook_utils import read_sqlite_file
import matplotlib.animation as animation

def rotate_vector(vector, angle_degrees):
    """ベクトルを指定角度だけ回転させる関数

    Args:
        vector: 回転させたいベクトル
        angle_degrees: 回転角度(度数)

    Returns:
        回転後のベクトル
    """
    # 度数法からラジアンに変換
    angle_radians = np.radians(angle_degrees)

    # 回転行列の定義
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]
    ])

    # 回転行列をベクトルに適用
    return np.dot(rotation_matrix, vector)

def branche_to_theta(brance_vector):
    """通路の回転角度を計算する
    Args:
        brance_vector: 枝分かれの通路のベクトル

    Returns:
        通路の回転角度(ラジアン)
    """
    # 左側の枝分かれ通路のベクトル
    left_branch_vector = [39, 64]

    # メインの通路のベクトル
    main_vector = rotate_vector(left_branch_vector, 0)

    # 通路の回転角度を計算
    theta = np.arctan2(main_vector[1], main_vector[0])
    return theta

#C^2→Rの写像
def vector_length(vector):
    return np.sqrt(vector[0]**2 + vector[1]**2)

#ベクトルの内積を与える関数
def vector_inner_product(x_vector,y_vector):
    return x_vector[0] * y_vector[0] + x_vector[1] * y_vector[1]

def add_vector(vector1,vector2):
    return [vector1[0] + vector2[0], vector1[1] + vector2[1]]

def multiply_vector(scalar,vector):
    return [vector[0] * scalar, vector[1] * scalar]

def set_agents_positions(agent_num,fixed_vector,about_direction_vector, base_point,distance_from_wall,distance_between_agents):
    """エージェント数、通路端ベクトル、通路回転角度、壁との距離、エージェント間の距離 → エージェント変位、通路長さ、通路ポリゴン

    Args:
        agent_num: エージェント数
        distance_from_wall: 壁との距離
        distance_between_agents: エージェント間の距離
        fixed_vector: 通路端ベクトル[方向は自由]
        about_direction_vector: 通路の固定ベクトルからどの方向に通路を設定するか(固定ベクトルに対しての方向によって決定する)
        base_point: 固定ベクトルの原点

    Returns:
        corridor_polygon: 通路のポリゴン
        positions: エージェントの座標[相対座標でなく、与えられた原点に基づいたベクトル]

    Cautions:
        fixed_vectorの向きは任意であるが、この向きにエージェントを詰めていくので、fixed_vectorの先側の壁からの距離はdistance_from_wallとの一致が保障されない
    """

    #必要なベクトルを生成
    x_e_vector = fixed_vector / vector_length(fixed_vector)
    direction_vector = list(np.array(about_direction_vector) - vector_inner_product(about_direction_vector, x_e_vector) * np.array(x_e_vector)) #演算にnumpyを利用
    z_e_vector = direction_vector / vector_length(direction_vector)

    #ベクトルが直交している確認
    pass # print(f"x_e_vector: {x_e_vector},z_e_vector: {z_e_vector},vector_inner_product(x_e_vector,z_e_vector): {vector_inner_product(x_e_vector,z_e_vector)}")


    #通路幅が狭すぎるの時のエラー
    if vector_length(fixed_vector) - 2 * distance_from_wall < 0:
        raise ValueError(f"通路幅が狭すぎます[最低でも{2 * distance_from_wall}が必要です]")

    #列数、行数を計算
    pass # print(f"vector_length(fixed_vector): {vector_length(fixed_vector)}, 2 * distance_from_wall: {2 * distance_from_wall}, distance_between_agents: {distance_between_agents}")
    row_number = math.ceil((vector_length(fixed_vector) - 2 * distance_from_wall) / distance_between_agents) #1列に詰め込める人数と等しい
    column_number = math.ceil(agent_num / (row_number + 1))

    #テストコード
    pass # print(f"row_number: {row_number}, column_number: {column_number}")

    #エージェントの座標を計算
    positions = []

    #1行目のエージェントの座標を設定
    first_row_positions = []
    for column_index in range(column_number):
        first_row_positions.append(list(np.array(base_point) + distance_from_wall * np.array(x_e_vector) + distance_from_wall * np.array(z_e_vector) + column_index * distance_between_agents * np.array(z_e_vector)))
    positions.extend(first_row_positions) #リストを結合[appendだと二重配列になる]

    #2行目以降のエージェントの座標を設定[1行目の座標にエージェント間距離*単位固定ベクトルを足していく]
    for row_index in range(1,row_number):
        #mapを使って全要素を移動させる
        row_k_positions = list(map(lambda x: list(np.array(x) + row_index * distance_between_agents * np.array(x_e_vector)), first_row_positions))
        #pass # print(f"row_k_positions: {row_k_positions}")

        #リストを結合
        positions.extend(row_k_positions)

    #ポリゴンの辺ベクトルを生成
    w_vector = list((2 * distance_from_wall + (column_number - 1) * distance_between_agents) * np.array(z_e_vector))
    corridor_polygon_coordinates = [
        base_point,
        list(np.array(base_point) + np.array(w_vector)),
        list(np.array(base_point) + np.array(w_vector) + np.array(fixed_vector)),
        list(np.array(base_point) + np.array(fixed_vector)),
    ]
    corridor_polygon = Polygon(corridor_polygon_coordinates)

    #テストコード
    #エージェントの座標が通路ポリゴンの中に含まれているか確認
    for position in positions:
        if not corridor_polygon.contains(Point(position)):
            #pass # print(f"Warning: Position {position} is outside the corridor polygon")
            pass

    return positions, corridor_polygon

def generate_agents_spawn_positions(distance_from_wall, distance_between_agents, fixed_vector, about_direction_vector, base_point):
    #シミュレーション内におけるエージェントのスポーン位置を指定する関数
    #必要なベクトルを生成
    x_e_vector = fixed_vector / vector_length(fixed_vector)
    direction_vector = list(np.array(about_direction_vector) - vector_inner_product(about_direction_vector, x_e_vector) * np.array(x_e_vector)) #演算にnumpyを利用
    z_e_vector = direction_vector / vector_length(direction_vector)

    #ベクトルが直交している確認
    pass # print(f"x_e_vector: {x_e_vector},z_e_vector: {z_e_vector},vector_inner_product(x_e_vector,z_e_vector): {vector_inner_product(x_e_vector,z_e_vector)}")


    #通路幅が狭すぎるの時のエラー
    if vector_length(fixed_vector) - 2 * distance_from_wall < 0:
        raise ValueError(f"通路幅が狭すぎます[最低でも{2 * distance_from_wall}が必要です]")

    #列数、行数を計算
    pass # print(f"vector_length(fixed_vector): {vector_length(fixed_vector)}, 2 * distance_from_wall: {2 * distance_from_wall}, distance_between_agents: {distance_between_agents}") #テストコード
    row_number = math.ceil((vector_length(fixed_vector) - 2 * distance_from_wall) / distance_between_agents) #1列に詰め込める人数と等しい

    return set_agents_positions(agent_num=row_number, distance_from_wall=distance_from_wall, distance_between_agents=distance_between_agents, fixed_vector=fixed_vector, about_direction_vector=about_direction_vector, base_point=base_point)

def geenrate_agents_waiting_positions(distance_from_wall, distance_between_agents, fixed_vector, about_direction_vector, area_length, base_point):
        #必要なベクトルを生成
    x_e_vector = fixed_vector / vector_length(fixed_vector)
    direction_vector = list(np.array(about_direction_vector) - vector_inner_product(about_direction_vector, x_e_vector) * np.array(x_e_vector)) #演算にnumpyを利用
    z_e_vector = direction_vector / vector_length(direction_vector)

    #ベクトルが直交している確認
    pass # print(f"x_e_vector: {x_e_vector},z_e_vector: {z_e_vector},vector_inner_product(x_e_vector,z_e_vector): {vector_inner_product(x_e_vector,z_e_vector)}")


    #通路幅が狭すぎるの時のエラー
    if vector_length(fixed_vector) - 2 * distance_from_wall < 0:
        raise ValueError(f"通路幅が狭すぎます[最低でも{2 * distance_from_wall}が必要です]")

    #列数、行数を計算
    pass # print(f"vector_length(fixed_vector): {vector_length(fixed_vector)}, 2 * distance_from_wall: {2 * distance_from_wall}, distance_between_agents: {distance_between_agents}")
    row_number = math.ceil((vector_length(fixed_vector) - 2 * distance_from_wall) / distance_between_agents) #1列に詰め込める人数と等しい
    column_number = math.ceil((area_length - 2 * distance_from_wall) / distance_between_agents)
    agent_num = row_number * column_number

    return set_agents_positions(agent_num=agent_num, distance_from_wall=distance_from_wall, distance_between_agents=distance_between_agents, fixed_vector=fixed_vector, about_direction_vector=about_direction_vector, base_point=base_point)[0]

def remove_duplicate_lines(lines_coordinates):
    # 線分の両端点をソートして正規化（順序を無視するため）
    normalized_lines = [tuple(sorted(line)) for line in lines_coordinates]

    # 出現回数をカウント
    from collections import Counter
    line_counts = Counter(normalized_lines)

    # 出現回数が1の線分だけ残す（重複を除いたもの）
    unique_lines = [list(line) for line, count in line_counts.items() if count == 1]

    return unique_lines

def distance_point_to_segment(x, y, x1, y1, x2, y2):
    ap = np.array([x - x1, y - y1]) #端点からのベクトル
    ab = np.array([x2 - x1, y2 - y1]) #方向ベクトル
    ab_len_squared = np.dot(ab, ab)
    if ab_len_squared == 0:
        return 10000  # AとBが同じ点
    t = np.dot(ap, ab) / ab_len_squared
    if 0 <= t <= 1:
        # 線分上に投影
        projection = np.array([x1, y1]) + t * ab
        return np.linalg.norm(projection - np.array([x, y]))
    else:
        return 10000 #垂線の足が線分上にない場合は莫大な値を返す[minで必ず無視されるように]

def filter_and_score_coordinates(polygon, coordinates, lines, designated_coordinate, alpha=0.1):
    #Polygon内にある点だけを残す
    valid_points = [pt for pt in coordinates if polygon.contains(Point(pt))]

    #各点にスコアを割り振る
    scored_points = []
    for (x, y) in valid_points:
        min_line_dist = min(
            distance_point_to_segment(x, y, x1, y1, x2, y2)
            for (x1, y1), (x2, y2) in lines
        )
        designated_dist = np.linalg.norm(np.array([x, y]) - np.array(designated_coordinate))
        score = min_line_dist + alpha * designated_dist
        scored_points.append(((x, y), score))

    #スコアが小さい順に並び替える
    scored_points.sort(key=lambda item: item[1])

    return [coord for coord, score in scored_points]

def line_to_wall_polygon_coordinates(line_coordinates, wall_thickness=0.1):
    # 線分の両端点
    p1 = np.array(line_coordinates[0])
    p2 = np.array(line_coordinates[1])
    # ベクトル
    vector = p2 - p1
    # 法線ベクトル
    normal = np.array([vector[1], -vector[0]])
    normal = normal / np.linalg.norm(normal) * (wall_thickness / 2)

    # 四角形の4点を反時計回りで生成
    q1 = p1 + normal
    q2 = p2 + normal
    q3 = p2 - normal
    q4 = p1 - normal

    wall_polygon_coordinates = [q1.tolist(), q2.tolist(), q3.tolist(), q4.tolist()]
    return wall_polygon_coordinates

#空間設定
def create_geometry():
    #基本的な歩行可能エリア[バグを避けるために、これらを、壁に変更することで処理する]
    default_geometry_coordinates = [
        [(19.35, 56.24),(24.38, 64.50),(49.15, 49.15),(41.80, 42.57)],
        [(41.80, 42.57),(43.86, 31.48),(48.63, 32.51),(49.15, 49.15)],
        [(48.63, 32.51),(49.15, 49.15),(50.31, 50.57),(58.31, 46.05)],
        [(48.63, 32.51),(51.21, 20.00),(67.08, 40.25),(58.31, 46.05)],
        [(51.21, 20.00),(56.63, 17.54),(69.40, 38.83),(67.08, 40.25)],
        [(58.31, 46.05),(67.08, 40.25),(88.03, 77.40),(72.11, 69.14)],
        [(72.11, 69.14),(73.01, 69.84), (79.78, 81.86),(79.08, 82.56),(85.14, 79.85),(88.03, 77.40)],
        [(79.08, 82.56),(85.14, 79.85),(89.78, 86.95),(85.40, 92.49)],
        [(79.85, 95.33),(79.85, 103.97),(87.98, 102.56),(85.40, 92.49)],
        [(102.81, 70.82),(106.04, 72.24),(111.71, 68.11),(110.04, 65.40)],
        [(89.78, 86.95),(92.88, 90.56),(106.04, 72.24),(102.81, 70.82)],
        [(85.40, 92.49),(92.75, 93.91),(92.88, 90.56),(89.78, 86.95)],
        [(85.40, 92.49),(87.98, 102.56),(97.78, 100.62),(92.75, 93.91)],
        [(87.98, 102.56),(97.78, 100.62),(114.42, 103.59),(114.04, 106.17)],
        [(114.42, 103.59),(97.78, 100.62),(92.75, 93.91),(92.88, 90.56),(106.04, 72.24),(111.71, 68.11)],
    ]

    #待機エージェントによるエリア
    #基本設定
    left_agent_num = 3000
    right_agent_num = 7000

    #左側の分岐エリア
    left_corridor_vector = np.array([24.381,64.5]) - np.array([19.35,56.244])
    left_base_point = np.array([19.35,56.244])

    #分岐エリアのポリゴンは利用しないが、かぶらないように,で区切る
    initial_sutudents_positions,left_corridor_polygon = generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=0.5, fixed_vector=left_corridor_vector, about_direction_vector=[1,0], base_point=left_base_point)
    #右側の分岐エリア
    right_corridor_vector = np.array([48.633,32.508]) - np.array([51.213,19.995])
    right_base_point = np.array([51.213,19.995])
    initial_parents_positions, right_corridor_polygon = generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=0.5, fixed_vector=right_corridor_vector, about_direction_vector=[1,0], base_point=right_base_point)

    #直線集合を構成
    lines_coordinates = []
    for i in range(len(default_geometry_coordinates)):
        for j in range(len(default_geometry_coordinates[i])):
            lines_coordinates.append([default_geometry_coordinates[i][j], default_geometry_coordinates[i][(j+1)%len(default_geometry_coordinates[i])]])

    #直線配列内で、重複している配列は、"どちらも除去"
    lines_coordinates = remove_duplicate_lines(lines_coordinates)

    pass # print(lines_coordinates) #テストコード

    #直線集合から、区切り壁のポリゴンを作成
    wall_polygons_coordinates = [line_to_wall_polygon_coordinates(line_coordinates,wall_thickness=0.1) for line_coordinates in lines_coordinates]

    # すべての多角形をPolygon化[壁の座標も含む]
    polygons = [Polygon(coords) for coords in default_geometry_coordinates + wall_polygons_coordinates]
    # 連結領域を作成
    outer_area = unary_union(polygons)

    # holesは今まで通り
    # 自己交差を避けるため、外周と穴を個別に処理
    exterior_coords = list(outer_area.exterior.coords)
    # 外周の座標を時計回りに並べ替え
    if not Polygon(exterior_coords).exterior.is_ccw:
        exterior_coords.reverse()
    # 穴の座標も時計回りに統一
    processed_holes = []
    for hole in wall_polygons_coordinates:
        hole_coords = list(hole)
        if Polygon(hole_coords).exterior.is_ccw:
            hole_coords.reverse()
        processed_holes.append(hole_coords)

    walkable_area_inside = Polygon(exterior_coords)

    #壁のポリゴンを取り除く
    wall_polygons = [Polygon(coords) for coords in wall_polygons_coordinates]
    all_wall_polygon = unary_union(wall_polygons)
    walkable_area = walkable_area_inside.difference(all_wall_polygon)

    # デバッグ用：壁のポリゴンと歩行可能エリアを視覚化
    pass # print("Visualizing walkable area and walls:")
    #visualize_geometry(walkable_area)

    #試験的に壁をなくす
    return walkable_area_inside, initial_sutudents_positions, initial_parents_positions

#エージェント設定
def define_agents_parameters(sutudents_positions, parents_positions):
    agents_parameters = []
    for i in range(len(sutudents_positions) + len(parents_positions)):
        if i < len(sutudents_positions):
            position = sutudents_positions[i]
        else:
            position = parents_positions[i - len(sutudents_positions)]
        agents_parameters.append(jps.AnticipationVelocityModelAgentParameters(
            journey_id=0,
            stage_id=0,
            desired_speed=4.0, #今はデフォルト値使ってるけど将来的には変更したほうが良い
            position=position,
            anticipation_time=1.0,
            reaction_time=0.3))
    return agents_parameters

def visualize_geometry(geometry):
    fig, ax = plt.subplots(figsize=(15, 15))
    
    if isinstance(geometry, list):
        for poly in geometry:
            # メインのPolygonを描画
            x, y = poly.exterior.xy
            ax.plot(x, y, 'b-', label='Polygon')
            ax.plot(x, y, 'bo')  # 青い点で端点を表示
            
            # 穴を描画
            for hole in poly.interiors:
                x, y = hole.xy
                ax.plot(x, y, 'r-', label='Hole')  # 赤い線で穴を表示
                ax.plot(x, y, 'ro')  # 赤い点で穴の端点を表示
                
    elif hasattr(geometry, 'geoms'):
        for poly in geometry.geoms:
            # メインのPolygonを描画
            x, y = poly.exterior.xy
            ax.plot(x, y, 'b-', label='Polygon')
            ax.plot(x, y, 'bo')
            
            # 穴を描画
            for hole in poly.interiors:
                x, y = hole.xy
                ax.plot(x, y, 'r-', label='Hole')
                ax.plot(x, y, 'ro')
                
    else:
        # メインのPolygonを描画
        x, y = geometry.exterior.xy
        ax.plot(x, y, 'b-', label='Polygon')
        ax.plot(x, y, 'bo')
        
        # 穴を描画
        for hole in geometry.interiors:
            x, y = hole.xy
            ax.plot(x, y, 'r-', label='Hole')
            ax.plot(x, y, 'ro')
    
    ax.set_aspect('equal')
    plt.show()

def validate_geometry(geometry):
    if not geometry.is_valid:
        fig, ax = plt.subplots()
        x, y = geometry.exterior.xy
        ax.plot(x, y, 'r-', label='Exterior')
        
        # 内部の穴があれば描画
        for interior in geometry.interiors:
            x, y = interior.xy
            ax.plot(x, y, 'b--', label='Interior')
            
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title('Invalid Geometry Visualization')
        ax.legend()
        plt.show()
        pass # print(f"Invalid geometry detected: {geometry}")
        #理由を表示
        pass # print(f"Invalid geometry reason: {explain_validity(geometry)}")
        # 無効なジオメトリを視覚化

        return False
    return True

def initialize_simulation(
    model, geometry, trajectory_file
):
    pass # print(f"シミュレーションの初期化を開始")
    if not validate_geometry(geometry):
        pass # print("Walkable area is invalid")
        # エラー処理
        pass

    simulation = jps.Simulation(
        model=model,
        geometry=geometry,
        dt=0.05,
        trajectory_writer=jps.SqliteTrajectoryWriter(
            output_file=pathlib.Path(trajectory_file), every_nth_frame=100
        ),
    )
    return simulation

def pass_switch_polygons():
    left_pass_switch_polygon = Polygon([
        (79.05, 82.56),
        (80.48, 84.96),
        (83.14, 83.07),
        (81.92, 79.87)
    ])
    right_pass_switch_polygon = Polygon([
        (81.92, 79.87),
        (83.14, 83.07),
        (86.74, 81.73),
        (85.18, 79.88),
    ])
    return left_pass_switch_polygon, right_pass_switch_polygon

def exit_switch_polygons():
    left_switch_polygon = Polygon([
        (85.40, 92.49),
        (86.39, 94.34),
        (89.61,91.99),
        (87.34, 89.12),
    ])
    validate_geometry(Polygon([
        (89.78, 86.95),  # Corrected the typo from 86,95 to 86.95
        (91.51, 90.577),
        (95.17, 86.31),
        (93.81, 81.78),
    ]))
    right_switch_polygon = Polygon([
        (89.78, 86.95),
        (91.51, 90.57),
        (89.61, 91.99),
        (87.34, 89.12),
    ])
    return left_switch_polygon, right_switch_polygon

def meeting_polygons_1():
    return [Polygon([
            (66.1, 33.2),
            (64.7, 31.1),
            (48.2, 39.7),
            (53.8, 41.7),
        ]),
        Polygon([
            (54.1, 48.6),
            (57.0, 46.9),
            (53.8, 41.7),
            (48.2, 39.7),
        ])],Polygon([ #諦めのポリゴン
        (67.08, 40.25),
        (88.03, 77.4),
        (84.655,79.63),
        (62.695,43.15),
    ])

def run_simulation(simulation, geometry, max_iterations,students_spawn_positions,parents_spawn_positions,student_num,parents_num,normal_student_journey_id,normal_parent_journey_id,switch_1,switch_2,agents_parameters, left_switch_polygon, right_switch_polygon, just_go_exit_1_journey_id, just_go_exit2_journey_id,go_exit1_waypoint,goal_1,goal_2, just_go_exit1_waypoint_journey_id, just_go_exit2_waypoint_journey_id,exit2_waypoint,meeting_rate_1,meeting_rate_2,meeting_radius,point_2,meeting_1_choice_waypoint,meeting_1_choice_journey_right_id,meeting_1_choice_journey_left_id,meeting_2_choice_waypoint,meeting_2_choice_journey_id):
    #meeting_rate_1,meeting_rate2　も受け取る
    pass # print(f"simulation.agent_count(): {simulation.agent_count()}")
    #保護者の出現地点を、ペアに分割[保護者がペアで出現できるようにするために]
    parents_pair_spawn_positions = [parents_spawn_positions[i:i+2] for i in range(0, len(parents_spawn_positions), 2)]
    #残りの出現数
    students_unspawned = student_num
    parents_unspawned = parents_num
    #固定定数
    distance_between_agents = 0.5
    #初期設定
    known_position = []
    student_id = 1
    parent_pair_id = 1
    #打ち消し用
    initial_agents_count = simulation.agent_count()
    just_only_go_exit1_journey = jps.JourneyDescription([
        goal_1,
    ])
    just_only_go_exit1_journey_id = simulation.add_journey(just_only_go_exit1_journey)

    #待ち合わせエージェントの作成
    #親の数が奇数ならエラー
    if parents_num % 2 == 1:
        raise ValueError("親の数が奇数です。")
    max_pair_num = min(student_num, parents_num // 2)

    #待ち合わせを行うグループ数を決定
    meeting_group_num_1 = math.floor(meeting_rate_1 * max_pair_num)
    meeting_group_num_2 = math.floor(meeting_rate_2 * max_pair_num)

    #待ち合わせグループの作成
    meeting_groups_parents_pairs = random.sample(range(1, parents_num // 2), meeting_group_num_1+meeting_group_num_2)
    meeting_groups_students_pairs = random.sample(range(1, student_num), meeting_group_num_1+meeting_group_num_2)

    #シャッフルしてから、分割
    random.shuffle(meeting_groups_parents_pairs) #再代入の必要はない
    meeting_groups_parents_pairs_1 = meeting_groups_parents_pairs[:meeting_group_num_1]
    meeting_groups_parents_pairs_2 = meeting_groups_parents_pairs[meeting_group_num_1:]

    random.shuffle(meeting_groups_students_pairs) #再代入の必要はない
    meeting_groups_students_pairs_1 = meeting_groups_students_pairs[:meeting_group_num_1]
    meeting_groups_students_pairs_2 = meeting_groups_students_pairs[meeting_group_num_1:]

    #辞書に変換
    group_ids_to_student_ids_1 = {
        group_id: student_id for group_id, student_id in enumerate(meeting_groups_students_pairs_1)
    }
    group_ids_to_student_ids_2 = {
        group_id: student_id for group_id, student_id in enumerate(meeting_groups_students_pairs_2)
    }
    group_ids_to_parent_ids_1 = {
        group_id: parent_id for group_id, parent_id in enumerate(meeting_groups_parents_pairs_1)
    }
    group_ids_to_parent_ids_2 = {
        group_id: parent_id for group_id, parent_id in enumerate(meeting_groups_parents_pairs_2)
    }

    #逆引きも作成
    student_ids_to_group_ids_1 = {
        student_id: group_id for group_id, student_id in enumerate(meeting_groups_students_pairs_1)
    }
    student_ids_to_group_ids_2 = {
        student_id: group_id for group_id, student_id in enumerate(meeting_groups_students_pairs_2)
    }
    parent_ids_to_group_ids_1 = {
        parent_id: group_id for group_id, parent_id in enumerate(meeting_groups_parents_pairs_1)
    }
    parent_ids_to_group_ids_2 = {
        parent_id: group_id for group_id, parent_id in enumerate(meeting_groups_parents_pairs_2)
    }


    #待ち合わせの失敗数の初期設定
    meeting_failure_count_1 = 0
    meeting_failure_count_2 = 0

    current_exited_agents = []

    #自作idと、シミュレーションidを対応させるためのdict
    groups_dict_1 = {}
    groups_dict_2 = {}

    #実験結果を保存するための変数
    result_groups_dict_1 = {}
    result_groups_dict_2 = {}

    #待ち合わせの選択を行うポリゴンを設定
    meeting_switch_one, meeting_giveup_one = meeting_polygons_1()

    #シミュレーション自動割り当てのidと、student_id,parent_idを対応させるためのdict
    meeting_1_auto_agent_id_dict = {}
    meeting_2_auto_agent_id_dict = {}

    while (
        (simulation.agent_count() > (student_num+parents_num)*0.1 or parents_unspawned != 0 or students_unspawned != 0) #スタックする人はどうしても数人は存在するから、それに対処
        and simulation.iteration_count() < max_iterations
    ):
        #まずは、生徒エージェントを自由に出現させる
        if students_unspawned > 0:
            for position in students_spawn_positions:
                #その地点周辺にエージェントが存在しないことを確認
                if len(list(simulation.agents_in_range(position, distance_between_agents))) == 0 and students_unspawned >= 1:
                    agent_id = simulation.add_agent(
                        agents_parameters(
                            journey_id=normal_student_journey_id, #journeyの調整は、スポーン後のイテレーションで行うので、ここでは行わない
                            stage_id=switch_1,
                            desired_speed=1.0,
                            position=position,
                        )
                    )
                    if student_id in student_ids_to_group_ids_1:
                        group_id = student_ids_to_group_ids_1[student_id]
                        if group_id not in groups_dict_1: #parent pairがスポーンしてないとき
                            groups_dict_1[group_id] = {"student":agent_id, "parents":None, "initial_meeting_position":None, "set":False, "target_agent":None, "none_target_agents_switched":[]}
                            pass # print(f"studentがスポーンしてないとき group_id: {group_id}")
                        else: #parent pairがスポーンしているとき
                            groups_dict_1[group_id]["student"] = agent_id
                        meeting_1_auto_agent_id_dict[agent_id] = group_id
                    if student_id in student_ids_to_group_ids_2:
                        group_id = student_ids_to_group_ids_2[student_id]
                        if group_id not in groups_dict_2: #parent pairがスポーンしてないとき
                            groups_dict_2[group_id] = {"student":agent_id, "parents":None, "initial_meeting_position":None, "set":False, "target_agent":None, "none_target_agents_switched":[]}
                        else: #parent pairがスポーンしているとき
                            groups_dict_2[group_id]["student"] = agent_id
                        meeting_2_auto_agent_id_dict[agent_id] = group_id
                    students_unspawned -= 1
                    student_id += 1
        #次に、保護者エージェントを、ペア"ごと"に出現させる
        if parents_unspawned > 0:
            for pair_position in parents_pair_spawn_positions:
                #奇数の場合、単独のエージェント[ペアでない]が生じる
                if len(pair_position) == 1:
                    pass
                else: #ペアで出現
                    spawn_able = len(list(simulation.agents_in_range(pair_position[0], distance_between_agents))) == 0 and len(list(simulation.agents_in_range(pair_position[1], distance_between_agents))) == 0
                    if spawn_able and parents_unspawned >= 2:
                        agent_id_1 = simulation.add_agent(
                            agents_parameters(
                                journey_id=normal_parent_journey_id,
                                stage_id=switch_2,
                                desired_speed=1.0,
                                position=pair_position[0],
                            )
                        )
                        agent_id_2 = simulation.add_agent(
                            agents_parameters(
                                journey_id=normal_parent_journey_id,
                                stage_id=switch_2,
                                desired_speed=1.0,
                                position=pair_position[1],
                            )
                        )
                        if parent_pair_id in parent_ids_to_group_ids_1:
                            group_id = parent_ids_to_group_ids_1[parent_pair_id]
                            if group_id not in groups_dict_1: #parent pairがスポーンしてないとき
                                groups_dict_1[group_id] = {"student":None, "parents":[agent_id_1,agent_id_2], "initial_meeting_position":None, "set":False, "target_agent":None, "none_target_agents_switched":[]}
                                pass # print(f"parent pairがスポーンしてないとき group_id: {group_id}")
                            else: #parent pairがスポーンしているとき
                                groups_dict_1[group_id]["parents"] = [agent_id_1,agent_id_2]
                            meeting_1_auto_agent_id_dict[agent_id_1] = group_id
                            meeting_1_auto_agent_id_dict[agent_id_2] = group_id
                        if parent_pair_id in parent_ids_to_group_ids_2:
                            group_id = parent_ids_to_group_ids_2[parent_pair_id]
                            if group_id not in groups_dict_2: #parent pairがスポーンしてないとき
                                groups_dict_2[group_id] = {"student":None, "parents":[agent_id_1,agent_id_2], "initial_meeting_position":None, "set":False, "target_agent":None, "none_target_agents_switched":[]}
                            else: #parent pairがスポーンしているとき
                                groups_dict_2[group_id]["parents"] = [agent_id_1,agent_id_2]
                            meeting_2_auto_agent_id_dict[agent_id_1] = group_id
                            meeting_2_auto_agent_id_dict[agent_id_2] = group_id
                        parents_unspawned -= 2
                        parent_pair_id += 1

        #2イテレーションごとに、エージェントの目標を切り替え
        right_pass_switch_polygon, left_pass_switch_polygon = pass_switch_polygons()
        meeting_switch_one_right, meeting_switch_one_left = meeting_switch_one[0], meeting_switch_one[1]

        #第二エリアでの待ち合わせで、場所の選択を開始
        if simulation.iteration_count() % 2 == 0:
            for agent in simulation.agents_in_polygon(left_switch_polygon):
                if agent in meeting_2_auto_agent_id_dict:
                    #最初のエージェントなら、場所を選択
                    if groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["set"] == False: #処理の頻度が高いと、通過中に複数回設定される可能性がある
                        simulation.switch_agent_journey(agent,meeting_2_choice_journey_id, meeting_2_choice_waypoint)
                        #更新処理
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["initial_meeting_position"] = None
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["target_agent"] = agent
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["set"] = True
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["meeting_initialized_iteration"] = simulation.iteration_count()
                    elif agent == groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["target_agent"]:
                        pass
                    else: #set=Trueなら、target_agentに向かう
                        meetpoint_waypoint = simulation.add_waypoint_stage(simulation.agent(groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["target_agent"]).position,1)
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            meetpoint_waypoint,
                        ]))
                        simulation.switch_agent_journey(agent,go_there_journey,meetpoint_waypoint)
                        #すでにこのエージェントを処理したことがある場合
                        if agent in groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["none_target_agents_switched"]:
                            pass
                        else: #初めて処理する場合
                            groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["none_target_agents_switched"].append(agent)
                else: #待ち合わせをしないエージェントなら、ただゴールに向かわせる
                    simulation.switch_agent_journey(agent,just_go_exit_1_journey_id,go_exit1_waypoint)
            for agent in simulation.agents_in_polygon(right_switch_polygon):
                if agent in meeting_2_auto_agent_id_dict:
                    #最初のエージェントなら、場所を選択
                    if groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["set"] == False: #処理の頻度が高いと、通過中に複数回設定される可能性がある
                        simulation.switch_agent_journey(agent,meeting_2_choice_journey_id, meeting_2_choice_waypoint)
                        #更新処理
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["initial_meeting_position"] = None
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["target_agent"] = agent
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["set"] = True
                        groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["meeting_initialized_iteration"] = simulation.iteration_count()
                    elif agent == groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["target_agent"]:
                        pass
                    else: #set=Trueなら、target_agentに向かう
                        meetpoint_waypoint = simulation.add_waypoint_stage(simulation.agent(groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["target_agent"]).position,1)
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            meetpoint_waypoint,
                        ]))
                        simulation.switch_agent_journey(agent,go_there_journey,meetpoint_waypoint)
                        #すでにこのエージェントを処理したことがある場合
                        if agent in groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["none_target_agents_switched"]:
                            pass
                        else: #初めて処理する場合
                            groups_dict_2[meeting_2_auto_agent_id_dict[agent]]["none_target_agents_switched"].append(agent)
                else: #待ち合わせをしないエージェントならただゴールに向かわせる
                    simulation.switch_agent_journey(agent,just_go_exit2_journey_id,goal_2)

        #第一エリアでの待ち合わせで、場所の選択を開始
        if simulation.iteration_count() % 20 == 0:
            for agent in simulation.agents_in_polygon(meeting_switch_one_right):
                #待ち合わせをするエージェントなら
                if agent in meeting_1_auto_agent_id_dict:
                    #最初のエージェントなら、場所を選択
                    if groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["set"] == False: #処理の頻度が高いと、通過中に複数回設定される可能性がある
                        simulation.switch_agent_journey(agent,meeting_1_choice_journey_right_id, meeting_1_choice_waypoint)
                        #更新処理
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["initial_meeting_position"] = None
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["target_agent"] = agent
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["set"] = True
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["meeting_initialized_iteration"] = simulation.iteration_count()
                    elif agent == groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["target_agent"]:
                        pass
                    else: #set=Trueなら、target_agentに向かう
                        meetpoint_waypoint = simulation.add_waypoint_stage(simulation.agent(groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["target_agent"]).position,1)
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            meetpoint_waypoint,
                        ]))
                        simulation.switch_agent_journey(agent,go_there_journey,meetpoint_waypoint)
                        #すでにこのエージェントを処理したことがある場合
                        if agent in groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["none_target_agents_switched"]:
                            pass
                        else: #初めて処理する場合
                            groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["none_target_agents_switched"].append(agent)
                else: #待ち合わせをしないエージェントならなにもしない
                    pass
            for agent in simulation.agents_in_polygon(meeting_switch_one_left):
                if agent in meeting_1_auto_agent_id_dict:
                    #最初のエージェントなら、場所を選択
                    if groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["set"] == False: #処理の頻度が高いと、通過中に複数回設定される可能性がある
                        # pass # print("2-1")
                        # pass # print(simulation.agent(agent).journey_id)
                        # pass # print(meeting_1_choice_journey_left_id)
                        # pass # print(meeting_1_choice_waypoint)
                        simulation.switch_agent_journey(agent,meeting_1_choice_journey_left_id, meeting_1_choice_waypoint)
                        #更新処理
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["initial_meeting_position"] = None
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["target_agent"] = agent
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["set"] = True
                        groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["meeting_initialized_iteration"] = simulation.iteration_count()
                    elif agent == groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["target_agent"]:
                        pass
                    else: #set=Trueなら、target_agentに向かう
                        #pass # print("2-2")
                        meetpoint_waypoint = simulation.add_waypoint_stage(simulation.agent(groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["target_agent"]).position,1)
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            meetpoint_waypoint,
                        ]))
                        simulation.switch_agent_journey(agent,go_there_journey,meetpoint_waypoint)
                        #すでにこのエージェントを処理したことがある場合
                        if agent in groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["none_target_agents_switched"]:
                            pass
                        else: #初めて処理する場合
                            groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["none_target_agents_switched"].append(agent)
                else: #待ち合わせをしないエージェントならなにもしない
                    pass

        #第二エリアでの待ち合わせで、ルーティングを切り替え[target_agentが流されるのに合わせて、合流点を変更][target_agentが場所についてしまっている場合に備えて、target-agentもinitial_meeting_positionに向かわせる]
        if simulation.iteration_count() % 20 == 0:
            for group in groups_dict_2.values():
                if group["set"] == True and len(group["none_target_agents_switched"]) > 0: #すでに処理をされてて、target_agent以外の処理済みエージェントがいる場合だけ処理する
                    for none_target_agent in group["none_target_agents_switched"]:
                        #現在のtarget_agentに向かう
                        meetpoint_waypoint = simulation.add_waypoint_stage(simulation.agent(group["target_agent"]).position,1)
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            meetpoint_waypoint,
                        ]))
                        simulation.switch_agent_journey(none_target_agent,go_there_journey,meetpoint_waypoint)
                    #target_agentの、到達による目的地変更を防ぐ
                    if group["initial_meeting_position"] is not None:
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            group["initial_meeting_position"],
                        ]))
                        simulation.switch_agent_journey(group["target_agent"],go_there_journey,group["initial_meeting_position"])
                    else: #target_agentのinitial_meeting_positionが未設定
                        if simulation.agent(group["target_agent"]).stage_id != meeting_2_choice_waypoint: #target_agentの待ち合わせ点が未設定でなければ、設定する
                            group["initial_meeting_position"] = simulation.agent(group["target_agent"]).stage_id

        #第一エリアでの待ち合わせで、ルーティングを切り替え[target_agentが流されるのに合わせて、合流点を変更][target_agentが場所についてしまっている場合に備えて、target-agentもinitial_meeting_positionに向かわせる]
        if simulation.iteration_count() % 20 == 0:
            for group in groups_dict_1.values():
                if group["set"] == True and len(group["none_target_agents_switched"]) > 0: #すでに処理をされてて、target_agent以外の処理済みエージェントがいる場合だけ処理する
                    for none_target_agent in group["none_target_agents_switched"]:
                        #現在のtarget_agentに向かう
                        meetpoint_waypoint = simulation.add_waypoint_stage(simulation.agent(group["target_agent"]).position,1)
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            meetpoint_waypoint,
                        ]))
                        simulation.switch_agent_journey(none_target_agent,go_there_journey,meetpoint_waypoint)
                    #target_agentの、到達による目的地変更を防ぐ
                    if group["initial_meeting_position"] is not None:
                        go_there_journey = simulation.add_journey(jps.JourneyDescription([
                            group["initial_meeting_position"],
                        ]))
                        simulation.switch_agent_journey(group["target_agent"],go_there_journey,group["initial_meeting_position"])
                    else: #target_agentのinitial_meeting_positionが未設定
                        if simulation.agent(group["target_agent"]).stage_id != meeting_1_choice_waypoint: #target_agentの待ち合わせ点が未設定でなければ、設定する
                            group["initial_meeting_position"] = simulation.agent(group["target_agent"]).stage_id

        #第一エリアでの待ち合わせで、諦める[agentの誰かが、流されすぎて、もうエリアで待ち合わせをできる見込みがない]
        if simulation.iteration_count() % 20 == 0:
            #第一エージェントの諦め処理はここに組み込む
            for agent in simulation.agents_in_polygon(left_pass_switch_polygon):
                if agent in meeting_1_auto_agent_id_dict and meeting_1_auto_agent_id_dict[agent] in groups_dict_1: #待ち合わせをするエージェントなら
                    pass # print(f"諦め処理が始まりました group_id: {meeting_1_auto_agent_id_dict[agent]}")
                    group = groups_dict_1[meeting_1_auto_agent_id_dict[agent]]
                    if group["set"] == True: #グループを削除し、target_agent,none_target_agents_switchedのjourneyをデフォルトのものに切り替える
                        #後の処理のために値を保存
                        student_id = group_ids_to_student_ids_1[meeting_1_auto_agent_id_dict[agent]]
                        parents_id = group_ids_to_parent_ids_1[meeting_1_auto_agent_id_dict[agent]]
                        #グループを削除[groups_dict_1から]
                        del groups_dict_1[meeting_1_auto_agent_id_dict[agent]]

                        #データをresult_groups_dict_1に保存
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[agent]] = group
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["status"] = "giveup"
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["giveup_iteration"] = simulation.iteration_count()

                        #none_target_agents_switchedの航路をデフォルトのものに切り替え
                        for none_target_agent in group["none_target_agents_switched"]: #target_agentは、ifなしの部分で処理できる[はず]
                            simulation.switch_agent_journey(none_target_agent,normal_student_journey_id,point_2)

                        #まだswitch_1を通過してない[けどスポーンしている]エージェントに、存在しない相手との待ち合わせが生じることがないように、判断基準のリスト[student_id,parent_id,student_id_reverse,parent_id_reverse,]から削除
                        if group["student"] is not None: #studentがスポーンしちゃってるとき→switchでの切り替えを防止するために、meeting_1_auto_agent_id_dictから削除
                            del meeting_1_auto_agent_id_dict[group["student"]]
                        else: #スポーンすらしてないときには、student_ids_to_group_ids_1から、stundent_id[自動idじゃない](group_idから、student_idを取得してやる)を削除
                            #student_id = group_ids_to_student_ids_1[meeting_1_auto_agent_id_dict[agent]] #student_ids_to_group_ids_1[group["group_id"]]
                            del student_ids_to_group_ids_1[student_id]

                        if group["parents"] is not None: #parentsがスポーンしちゃってるとき→switchでの切り替えを解除するために、meeting_1_auto_agent_id_dictから削除
                            del meeting_1_auto_agent_id_dict[group["parents"][0]]
                            del meeting_1_auto_agent_id_dict[group["parents"][1]]
                        else: #スポーンすらしてないときには、parents_ids_1から、parents_id[自動idじゃない](group_idから、parents_idを取得してやる)を削除
                            #parents_id = group_ids_to_parent_ids_1[meeting_1_auto_agent_id_dict[agent]] #parent_ids_to_group_ids_1[group["group_id"]]
                            del parent_ids_to_group_ids_1[parents_id]
                    else: #まだ誰もswitch_1を通過してないのに、あきらめてたらおかしい
                        pass
                        #raise ValueError("set=Falseなのに、あきらめてたらおかしい")
                    meeting_failure_count_1 += 1
                simulation.switch_agent_journey(agent,just_go_exit1_waypoint_journey_id,go_exit1_waypoint)
            for agent in simulation.agents_in_polygon(right_pass_switch_polygon):
                if agent in meeting_1_auto_agent_id_dict and meeting_1_auto_agent_id_dict[agent] in groups_dict_1: #待ち合わせをするエージェントなら
                    pass # print(f"諦め処理が始まりました group_id: {meeting_1_auto_agent_id_dict[agent]}")
                    group = groups_dict_1[meeting_1_auto_agent_id_dict[agent]]
                    if group["set"] == True: #グループを削除し、target_agent,none_target_agents_switchedのjourneyをデフォルトのものに切り替える
                        #後の処理のために値を保存
                        student_id = group_ids_to_student_ids_1[meeting_1_auto_agent_id_dict[agent]]
                        parents_id = group_ids_to_parent_ids_1[meeting_1_auto_agent_id_dict[agent]]
                        #グループを削除[groups_dict_1から]
                        del groups_dict_1[meeting_1_auto_agent_id_dict[agent]]

                        #データをresult_groups_dict_1に保存
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[agent]] = group
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["status"] = "giveup"
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[agent]]["giveup_iteration"] = simulation.iteration_count()

                        #none_target_agents_switchedの航路をデフォルトのものに切り替え
                        for none_target_agent in group["none_target_agents_switched"]: #target_agentは、ifなしの部分で処理できる[はず]
                            simulation.switch_agent_journey(none_target_agent,normal_student_journey_id,point_2)

                        #まだswitch_1を通過してない[けどスポーンしている]エージェントに、存在しない相手との待ち合わせが生じることがないように、判断基準のリスト[student_id,parent_id,student_id_reverse,parent_id_reverse,]から削除
                        if group["student"] is not None: #studentがスポーンしちゃってるとき→switchでの切り替えを防止するために、meeting_1_auto_agent_id_dictから削除
                            del meeting_1_auto_agent_id_dict[group["student"]]
                        else: #スポーンすらしてないときには、student_ids_to_group_ids_1から、stundent_id[自動idじゃない](group_idから、student_idを取得してやる)を削除
                            #student_id = group_ids_to_student_ids_1[meeting_1_auto_agent_id_dict[agent]] #student_ids_to_group_ids_1[group["group_id"]]
                            del student_ids_to_group_ids_1[student_id]

                        if group["parents"] is not None: #parentsがスポーンしちゃってるとき→switchでの切り替えを解除するために、meeting_1_auto_agent_id_dictから削除
                            del meeting_1_auto_agent_id_dict[group["parents"][0]]
                            del meeting_1_auto_agent_id_dict[group["parents"][1]]
                        else: #スポーンすらしてないときには、parents_ids_1から、parents_id[自動idじゃない](group_idから、parents_idを取得してやる)を削除
                            #parents_id = group_ids_to_parent_ids_1[meeting_1_auto_agent_id_dict[agent]] #parent_ids_to_group_ids_1[group["group_id"]]
                            del parent_ids_to_group_ids_1[parents_id]
                    else: #まだ誰もswitch_1を通過してないのに、あきらめてたらおかしい
                        pass
                        #raise ValueError("set=Falseなのに、あきらめてたらおかしい")
                    meeting_failure_count_1 += 1
                simulation.switch_agent_journey(agent,just_go_exit2_waypoint_journey_id,exit2_waypoint)

        #待ち合わせ完了の場合を処理[第二エリア]
        if simulation.iteration_count() % 20 == 0:
            groups_to_delete = [] #ループ中のサイズ変更が禁止されているため
            for group in groups_dict_2.values():
                if len(group["none_target_agents_switched"]) == 2: #待ち合わせするためのエージェントが3人いるときだけ
                    #待ち合わせ完了してるかどうかの判定実施
                    target_nearby_agents = simulation.agents_in_range(simulation.agent(group["target_agent"]).position, meeting_radius)
                    if group["none_target_agents_switched"][0] in target_nearby_agents and group["none_target_agents_switched"][1] in target_nearby_agents: #どっちも、target_agentからmeeting_radius以内にいるなら、待ち合わせ完了
                        pass # print(f"待ち合わせ完了2 group_id: {meeting_2_auto_agent_id_dict[group['target_agent']]}")
                        #グループを削除
                        groups_to_delete.append(meeting_2_auto_agent_id_dict[group["target_agent"]])

                        #データをresult_groups_dict_2に保存
                        result_groups_dict_2[meeting_2_auto_agent_id_dict[group["target_agent"]]] = group
                        result_groups_dict_2[meeting_2_auto_agent_id_dict[group["target_agent"]]]["status"] = "success"
                        result_groups_dict_2[meeting_2_auto_agent_id_dict[group["target_agent"]]]["meeting_iteration"] = simulation.iteration_count()

                        #グループのすべてのエージェントの航路をデフォルトのものに切り替え
                        if simulation.agent(group["target_agent"]).position[1] > 91: #y座標が91を超えていたら、goal_1に向かわせる
                            for none_target_agent in group["none_target_agents_switched"]:
                                simulation.switch_agent_journey(none_target_agent,just_only_go_exit1_journey_id,goal_1)
                            simulation.switch_agent_journey(group["target_agent"],just_only_go_exit1_journey_id,goal_1)
                        else:
                            for none_target_agent in group["none_target_agents_switched"]:
                                simulation.switch_agent_journey(none_target_agent,just_go_exit2_journey_id,goal_2)
                            simulation.switch_agent_journey(group["target_agent"],just_go_exit2_journey_id,goal_2)

                        #すべてのエージェントを、meeting_1_auto_agent_id_dictから削除
                        for none_target_agent in group["none_target_agents_switched"]:
                            del meeting_2_auto_agent_id_dict[none_target_agent]
                        del meeting_2_auto_agent_id_dict[group["target_agent"]]

                        #まだ誰も通過していないような場合は存在しないから、処理は不要
            for delete_group_id in groups_to_delete:
                del groups_dict_2[delete_group_id]

        #待ち合わせ完了の場合を処理[第一エリア]
        if simulation.iteration_count() % 20 == 0:
            groups_to_delete = [] #ループ中のサイズ変更が禁止されているため
            for group in groups_dict_1.values():
                if len(group["none_target_agents_switched"]) == 2: #待ち合わせするためのエージェントが3人いるときだけ
                    #待ち合わせ完了してるかどうかの判定実施
                    target_nearby_agents = simulation.agents_in_range(simulation.agent(group["target_agent"]).position, meeting_radius)
                    if group["none_target_agents_switched"][0] in target_nearby_agents and group["none_target_agents_switched"][1] in target_nearby_agents: #どっちも、target_agentからmeeting_radius以内にいるなら、待ち合わせ完了
                        pass # print(f"待ち合わせ完了 group_id: {meeting_1_auto_agent_id_dict[group['target_agent']]}")
                        #グループを削除
                        groups_to_delete.append(meeting_1_auto_agent_id_dict[group["target_agent"]])

                        #データをresult_groups_dict_1に保存
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[group["target_agent"]]] = group
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[group["target_agent"]]]["status"] = "success"
                        result_groups_dict_1[meeting_1_auto_agent_id_dict[group["target_agent"]]]["meeting_iteration"] = simulation.iteration_count()

                        #グループのすべてのエージェントの航路をデフォルトのものに切り替え
                        for none_target_agent in group["none_target_agents_switched"]:
                            simulation.switch_agent_journey(none_target_agent,normal_student_journey_id,point_2)
                        simulation.switch_agent_journey(group["target_agent"],normal_student_journey_id,point_2)

                        #すべてのエージェントを、meeting_1_auto_agent_id_dictから削除
                        for none_target_agent in group["none_target_agents_switched"]:
                            del meeting_1_auto_agent_id_dict[none_target_agent]
                        del meeting_1_auto_agent_id_dict[group["target_agent"]]

                        #まだ誰も通過していないような場合は存在しないから、処理は不要
            for delete_group_id in groups_to_delete:
                del groups_dict_1[delete_group_id]

        simulation.iterate()
        #進捗表示機能
        if simulation.iteration_count() % 100 == 0:
            pass # print(f"Iteration: {simulation.iteration_count()} agent_count: {simulation.agent_count()} students_unspawned: {students_unspawned} parents_unspawned: {parents_unspawned} agents_exited:{student_num+parents_num-simulation.agent_count() - students_unspawned - parents_unspawned + initial_agents_count}") #, end="\r")
    #残ったグループについて登録
    for group_id, group in groups_dict_1.items():
        result_groups_dict_1[group_id] = group
        result_groups_dict_1[group_id]["status"] = "simulation end"
        result_groups_dict_1[group_id]["giveup_time"] = simulation.elapsed_time() * 20
    for group_id, group in groups_dict_2.items():
        result_groups_dict_2[group_id] = group
        result_groups_dict_2[group_id]["status"] = "simulation end"
        result_groups_dict_2[group_id]["giveup_time"] = simulation.elapsed_time() * 20
    pass # print(f"Evacuation time: {simulation.elapsed_time():.2f} s")
    pass # print(f"meeting_failure_count_1: {meeting_failure_count_1}")
    return simulation.elapsed_time(), result_groups_dict_1,result_groups_dict_2


#シミュレーション実行
def main(max_iterations,meeting_rate_1,meeting_rate_2,trajectory_file):
    geometry, sutudents_positions, parents_positions = create_geometry()
    #agents_parameters = define_agents_parameters(sutudents_positions, parents_positions)
    model = jps.CollisionFreeSpeedModel() #AnticipationVelocityModel()

    #シミュレーションの初期化を組み込んだ際に削除すること
    positions = [position for position in sutudents_positions + parents_positions]
    speeds = [1.0 for _ in range(len(sutudents_positions) + len(parents_positions))]
    agents_parameters = jps.CollisionFreeSpeedModelAgentParameters #AnticipationVelocityModelAgentParameters

    #左側の待機位置1
    lines_coordinates_1_1 = [[(58.31,46.05),(79.78, 81.86)]]
    left_corridor_vector_1_1 = np.array([79.78, 81.86]) - np.array([58.31,46.05])
    left_base_point_1_1 = np.array([58.31,46.05])
    designated_coordinate = (55.53,43.08)
    polygon_1_1 = Polygon([(58.31,46.05),(79.78, 81.86),(84.655,79.63),(62.695,43.15)])
    generated_coordinates_1_1 = geenrate_agents_waiting_positions(distance_from_wall=0.5, distance_between_agents=1.0, fixed_vector=left_corridor_vector_1_1, about_direction_vector=[1,0], area_length=26.8, base_point=left_base_point_1_1)
    waiting_points_1_1 = filter_and_score_coordinates(polygon_1_1,generated_coordinates_1_1,lines_coordinates_1_1,designated_coordinate,alpha=0.01)

    #左側の待機位置2
    lines_coordinates_1_2 = [[(67.08, 40.25),(89.53, 77.4)]]
    left_corridor_vector_1_2 = np.array([89.53, 77.4]) - np.array([67.08, 40.25])
    left_base_point_1_2 = np.array([67.08, 40.25])
    designated_coordinate = (62.695,43.15)
    polygon_1_2 = Polygon([(67.08, 40.25),(88.03, 77.4),(84.655,79.63),(62.695,43.15)])
    generated_coordinates_1_2 = geenrate_agents_waiting_positions(distance_from_wall=0.5, distance_between_agents=1.0, fixed_vector=left_corridor_vector_1_2, about_direction_vector=[-1,0], area_length=26.8, base_point=left_base_point_1_2)
    waiting_points_1_2 = filter_and_score_coordinates(polygon_1_2,generated_coordinates_1_2,lines_coordinates_1_2,designated_coordinate,alpha=0.01)

    #右側の待機位置
    lines_coordinates_2 = [
        [(114.42, 103.59),(97.78, 100.62)],
        [(97.78, 100.62),(92.75, 93.91)],
        [(92.75, 93.91),(92.88, 90.56)],
        [(92.88, 90.56),(106.04, 72.24)],
        [(106.04, 72.24),(111.71, 68.11)],
    ]
    designated_coordinate = (92.88,90.56) #中心部に近い点の一つ
    left_corridor_vector = np.array([86.37, 112.78]) - np.array([88.84,62.21])
    left_base_point = np.array([88.84,62.21])
    polygon_2 = Polygon([(114.42, 103.59),(97.78, 100.62),(92.75, 93.91),(92.88, 90.56),(106.04, 72.24),(111.71, 68.11)])
    generated_coordinates_2 = geenrate_agents_waiting_positions(distance_from_wall=0.5, distance_between_agents=0.5, fixed_vector=left_corridor_vector, about_direction_vector=[1,0], area_length=30, base_point=left_base_point)
    waiting_points_2 = filter_and_score_coordinates(polygon_2,generated_coordinates_2,lines_coordinates_2,designated_coordinate)

    pass # print(f"シミュレーション実行段階に到達")
    simulation = initialize_simulation(model, geometry, trajectory_file)

    #journeyを設定
    #goalを作成
    goal_1 = simulation.add_exit_stage([
        (113.792, 106.106),
        (113.502, 106.082),
        (113.022, 103.660),
        (114.308, 103.700)
    ])
    pass # print(f"goal_1 stage_id: {goal_1}")
    goal_2 = simulation.add_exit_stage([
        (109.723,69.131), (110.907,68.319),(109.883, 65.822), (108.334, 67.112),
    ])
    pass # print(f"goal_2 stage_id: {goal_2}")
    #非待ち合わせ生徒のjourney
    vector_1_1 = np.array([58.31, 46.05]) - np.array([62.695, 43.15])
    base_point_1_1 = np.array([62.695, 43.15])
    switch_1 = simulation.add_waypoint_stage(
        (22.45, 59.67),6
    )
    #Waypointを一つに固定してみる
    point_0 = simulation.add_waypoint_stage((48.3, 40.17),10)
    point_1 = simulation.add_waypoint_stage((62.695, 43.15), 7)
    point_2 = simulation.add_waypoint_stage((81.84, 79.13), 7)
    point_3 = simulation.add_waypoint_stage((96.3, 94.3), 8)

    pass # print(f"switch_1 stage_id: {switch_1}")
    vector_1_0 = np.array([54.03, 47.05]) - np.array([48.3, 40.17])
    base_point_1_0 = np.array([48.3, 40.17])
    points_0 = [simulation.add_waypoint_stage(coordinate,10) for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_0), fixed_vector=vector_1_0, about_direction_vector=[1,0], base_point=base_point_1_0)[0]]
    point_0_coordinates = [coordinate for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_0), fixed_vector=vector_1_0, about_direction_vector=[1,0], base_point=base_point_1_0)[0]]
    points_1 = [simulation.add_waypoint_stage(coordinate,7) for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_1), fixed_vector=vector_1_1, about_direction_vector=[1,0], base_point=base_point_1_1)[0]]
    pass # print(f"points_1 stage_ids: {points_1}")
    point_1_coordinates = [coordinate for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_1), fixed_vector=vector_1_1, about_direction_vector=[1,0], base_point=base_point_1_1)[0]]
    vector_1_2 = np.array([79.08, 82.56]) - np.array([81.84, 79.13])#np.array([81.82, 81.48])
    base_point_1_2 = np.array([81.84, 79.13])#np.array([81.82, 81.48])
    points_2 = [simulation.add_waypoint_stage(coordinate,7) for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_2), fixed_vector=vector_1_2, about_direction_vector=[1,0], base_point=base_point_1_2)[0]]
    pass # print(f"points_2 stage_ids: {points_2}")
    point_2_coordinates = [coordinate for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_2), fixed_vector=vector_1_2, about_direction_vector=[1,0], base_point=base_point_1_2)[0]]
    vector_1_3 = np.array([90.21, 90.02]) - np.array([96.3, 94.3]) #np.array([89.3, 93.1]) #np.array([87.21, 94.3])
    base_point_1_3 = np.array([96.3, 94.3]) # np.array([89.3, 93.1]) # np.array([87.21, 94.3])
    points_3 = [simulation.add_waypoint_stage(coordinate,8) for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_3), fixed_vector=vector_1_3, about_direction_vector=[1,0], base_point=base_point_1_3)[0]]
    pass # print(f"points_3 stage_ids: {points_3}")
    point_3_coordinates = [coordinate for coordinate in generate_agents_spawn_positions(distance_from_wall=0.5, distance_between_agents=vector_length(vector_1_3), fixed_vector=vector_1_3, about_direction_vector=[1,0], base_point=base_point_1_3)[0]]
    # normal_student_journey = jps.JourneyDescription([
    #     switch_1,*points_0,*points_1,*points_2,*points_3,goal_1,goal_2
    # ])
    normal_student_journey = jps.JourneyDescription([
        switch_1,point_1,point_2,point_3,goal_1,goal_2
    ])
    normal_student_journey.set_transition_for_stage(
        switch_1,
        jps.Transition.create_fixed_transition(
            point_1
        )
    )
    normal_student_journey.set_transition_for_stage(
        point_1,
        jps.Transition.create_fixed_transition(
            point_2
        )
    )
    normal_student_journey.set_transition_for_stage(
        point_2,
        jps.Transition.create_fixed_transition(
            point_3
        )
    )
    normal_student_journey.set_transition_for_stage(
        point_3,
        jps.Transition.create_least_targeted_transition(
            [goal_1,goal_2]
        )
    )

    #自動切換えの作成
    go_exit1_waypoint = simulation.add_waypoint_stage((89.6, 101.), 3)
    just_go_exit_1_journey = jps.JourneyDescription([
        go_exit1_waypoint,goal_1
    ])
    just_go_exit_1_journey.set_transition_for_stage(
        go_exit1_waypoint,
        jps.Transition.create_fixed_transition(
            goal_1
        )
    )
    just_go_exit_1_journey_id = simulation.add_journey(just_go_exit_1_journey)

    just_go_exit2_journey = jps.JourneyDescription([
        goal_2
    ])
    just_go_exit2_journey_id = simulation.add_journey(just_go_exit2_journey)

    #通過時の通過ルートを作成
    just_go_exit1_waypoint_journey = jps.JourneyDescription([
        go_exit1_waypoint
    ])
    just_go_exit1_waypoint_journey_id = simulation.add_journey(just_go_exit1_waypoint_journey)

    exit2_waypoint = simulation.add_waypoint_stage((93.6, 92.1), 3)
    just_go_exit2_waypoint_journey = jps.JourneyDescription([
        exit2_waypoint
    ])
    just_go_exit2_waypoint_journey_id = simulation.add_journey(just_go_exit2_waypoint_journey)

    normal_student_journey_id = simulation.add_journey(normal_student_journey)
    pass # print(f"normal_student_journey_id: {normal_student_journey_id}")
    #この方式だと、並んでいる人がいるときに大変なことになる可能性が存在する
    #非待ち合わせの保護者のjourney
    switch_2 = simulation.add_waypoint_stage(
        (50.21, 25.82),10
    )
    pass # print(f"switch_2 stage_id: {switch_2}")
    points_2 = [simulation.add_waypoint_stage(coordinate,5) for coordinate in waiting_points_1_2]
    pass # print(f"parent points_2 stage_ids: {points_2}")
    points_3 = [simulation.add_waypoint_stage(coordinate,8) for coordinate in waiting_points_2]
    pass # print(f"parent points_3 stage_ids: {points_3}")
    point_1_2 = simulation.add_waypoint_stage((73.3,53.2), 7)
    normal_parent_journey = jps.JourneyDescription([
        switch_2,point_1_2,point_2,point_3,goal_1,goal_2
    ])
    normal_parent_journey.set_transition_for_stage(
        switch_2,
        jps.Transition.create_fixed_transition(
            point_1_2
        )
    )
    normal_parent_journey.set_transition_for_stage(
        point_1_2,
        jps.Transition.create_fixed_transition(
            point_2
        )
    )
    normal_parent_journey.set_transition_for_stage(
        point_2,
        jps.Transition.create_fixed_transition(
            point_3
        )
    )
    normal_parent_journey.set_transition_for_stage(
        point_3,
        jps.Transition.create_least_targeted_transition(
            [goal_1,goal_2]
        )
    )

    #待ち合わせ1のための経路
    meeting_1_choice_waypoint = point_1
    waiting_waypoint_1_1 = [simulation.add_waypoint_stage(waiting_point,1) for waiting_point in waiting_points_1_1]
    meeting_1_choice_journey_left = jps.JourneyDescription([
        meeting_1_choice_waypoint, *waiting_waypoint_1_1
    ])
    meeting_1_choice_journey_left.set_transition_for_stage(
        meeting_1_choice_waypoint,
        jps.Transition.create_least_targeted_transition(
            waiting_waypoint_1_1
        )
    )
    waiting_waypoint_1_2 = [simulation.add_waypoint_stage(waiting_point,1) for waiting_point in waiting_points_1_2]
    meeting_1_choice_journey_right = jps.JourneyDescription([
        meeting_1_choice_waypoint, *waiting_waypoint_1_2
    ])
    meeting_1_choice_journey_right.set_transition_for_stage(
        meeting_1_choice_waypoint,
        jps.Transition.create_least_targeted_transition(
            waiting_waypoint_1_2
        )
    )

    meeting_1_choice_journey_left_id = simulation.add_journey(meeting_1_choice_journey_left)
    meeting_1_choice_journey_right_id = simulation.add_journey(meeting_1_choice_journey_right)

    meeting_2_choice_waypoint = simulation.add_waypoint_stage((91.3,94.2), 7)
    meeting_2_waypoint_1 = [simulation.add_waypoint_stage(waiting_point,1) for waiting_point in waiting_points_2]
    meeting_2_choice_journey = jps.JourneyDescription([
        meeting_2_choice_waypoint,
        *meeting_2_waypoint_1
    ])
    meeting_2_choice_journey.set_transition_for_stage(
        meeting_2_choice_waypoint,
        jps.Transition.create_least_targeted_transition(
            meeting_2_waypoint_1
        )
    )
    meeting_2_choice_journey_id = simulation.add_journey(meeting_2_choice_journey)
    normal_parent_journey_id = simulation.add_journey(normal_parent_journey)

    #スイッチポリゴンを作成
    left_switch_polygon, right_switch_polygon = exit_switch_polygons()

    #いくつかの初期変数の値を設定
    meeting_radius = 3

    evacuation_time, result_groups_dict_1, result_groups_dict_2 = run_simulation(simulation=simulation, geometry=geometry, max_iterations=max_iterations,students_spawn_positions=sutudents_positions,parents_spawn_positions=parents_positions,student_num=3200,parents_num=5300,normal_student_journey_id=normal_student_journey_id,normal_parent_journey_id=normal_parent_journey_id,switch_1=switch_1,switch_2=switch_2,agents_parameters=agents_parameters,left_switch_polygon=left_switch_polygon,right_switch_polygon=right_switch_polygon, just_go_exit_1_journey_id=just_go_exit_1_journey_id, just_go_exit2_journey_id=just_go_exit2_journey_id,go_exit1_waypoint=go_exit1_waypoint,goal_2=goal_2, just_go_exit1_waypoint_journey_id=just_go_exit1_waypoint_journey_id, just_go_exit2_waypoint_journey_id=just_go_exit2_waypoint_journey_id,exit2_waypoint=exit2_waypoint,meeting_rate_1=meeting_rate_1,meeting_rate_2=meeting_rate_2,meeting_radius=meeting_radius,point_2=point_2,meeting_1_choice_waypoint=meeting_1_choice_waypoint,meeting_1_choice_journey_right_id=meeting_1_choice_journey_right_id, meeting_1_choice_journey_left_id=meeting_1_choice_journey_left_id,meeting_2_choice_waypoint=meeting_2_choice_waypoint,meeting_2_choice_journey_id=meeting_2_choice_journey_id,goal_1=goal_1)
    return evacuation_time, result_groups_dict_1, result_groups_dict_2

if __name__ == "__main__": #もっといろいろリセットしてから再実行
    main()