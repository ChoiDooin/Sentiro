
import os
import random
import pandas as pd
import geopandas as gpd
import networkx as nx
from shapely.geometry import (
    Point, LineString, MultiLineString, Polygon, MultiPolygon, GeometryCollection
)

from shapely.ops import unary_union, nearest_points

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGraphicsView, QGraphicsScene, QGraphicsLineItem,
    QGraphicsPixmapItem, QGraphicsPolygonItem, QGraphicsPathItem, QToolTip,
    QGraphicsEllipseItem, QLabel, QPushButton, QHBoxLayout, QSizePolicy
)
from PyQt6.QtGui import (
    QPainter, QPainterPath, QPen, QBrush, QColor, QPixmap, QPolygonF,QFont
)
from PyQt6.QtCore import QPointF, Qt, QTimer



from robot_point_importer import start_x, start_y
from robot_destination_page import RobotDestinationPage


MIN_X, MIN_Y = -1.15, -18.8
MAX_X, MAX_Y = 8.5, -9.0

def load_layers(folder: str, floor_code: str = "1F") -> dict:
    layers = {}
    layers['outline'] = gpd.read_file(os.path.join(folder, 'outline_pgm.geojson'))
    layers['doors'] = gpd.read_file(os.path.join(folder, 'doors_pgm.geojson'))
    stair_elev = gpd.read_file(os.path.join(folder, 'stair_n_elevator_pgm.geojson'))
    layers['stair'] = stair_elev[stair_elev['id'] == 0].reset_index(drop=True)
    layers['elevator'] = stair_elev[stair_elev['id'] == 1].reset_index(drop=True)
    shop_path = os.path.join(folder, f'shops_pgm_{floor_code}.geojson')
    if os.path.exists(shop_path):
        shops = gpd.read_file(shop_path)
        layers['room_n_stair'] = pd.concat([shops, layers['stair'], layers['elevator']], ignore_index=True)
    else:
        layers['room_n_stair'] = pd.concat([layers['stair'], layers['elevator']], ignore_index=True)
    path_path = os.path.join(folder, f'path_pgm_{floor_code}.geojson')
    if os.path.exists(path_path):
        layers['path'] = gpd.read_file(path_path)
    else:
        layers['path'] = gpd.GeoDataFrame(columns=['geometry'], geometry='geometry')
    return layers

def build_graph(path_gdf: gpd.GeoDataFrame) -> nx.Graph:
    G = nx.Graph()
    union = unary_union(path_gdf.geometry)

    # geometry를 리스트로 정규화
    if isinstance(union, (MultiLineString, GeometryCollection)):
        lines = [geom for geom in union.geoms if isinstance(geom, LineString)]
    elif isinstance(union, LineString):
        lines = [union]
    else:
        lines = []

    # 간선 추가
    for line in lines:
        coords = list(line.coords)
        for u, v in zip(coords[:-1], coords[1:]):
            dist = Point(u).distance(Point(v))
            G.add_edge((u[0], u[1]), (v[0], v[1]), weight=dist)

    return G


def insert_projection(G: nx.Graph, proj_pt: Point) -> tuple:
    best_edge, best_dist = None, float('inf')
    for u, v in G.edges():
        line = LineString([u, v])
        d = line.distance(proj_pt)
        if d < best_dist:
            best_dist, best_edge = d, (u, v, line)
    if best_edge is None:
        return None
    u, v, line = best_edge
    proj_on = line.interpolate(line.project(proj_pt))
    node = (proj_on.x, proj_on.y)
    G.remove_edge(u, v)
    du = Point(u).distance(proj_on)
    dv = Point(v).distance(proj_on)
    G.add_node(node)
    G.add_edge(u, node, weight=du)
    G.add_edge(node, v, weight=dv)
    return node

class InfoPolygonItem(QGraphicsPolygonItem):
    
    def __init__(self, polygon: QPolygonF, info: str, basepage, map_widget, geom):
        super().__init__(polygon)
        self.basepage = basepage
        self.map_widget = map_widget 
        self.info = info
        self.geom = geom
        self.setAcceptHoverEvents(True)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)

    def mousePressEvent(self, e):
        InfoPage(self.info, self.basepage, self.geom.centroid)
        super().mousePressEvent(e)


    def hoverEnterEvent(self, e):
        QToolTip.showText(e.screenPos(), str(self.info))
        super().hoverEnterEvent(e)

    def hoverLeaveEvent(self, e):
        QToolTip.hideText()
        super().hoverLeaveEvent(e)

 

class InfoEllipseItem(QGraphicsEllipseItem):
    def __init__(self, x: float, y: float, w: float, h: float, info: str, map_widget, geom):
        super().__init__(x, y, w, h)
        self.map_widget = map_widget
        self.geom = geom
        self.info = info
        self.setAcceptHoverEvents(True)

    def hoverEnterEvent(self, e):
        QToolTip.showText(e.screenPos(), str(self.info))
        super().hoverEnterEvent(e)

    def hoverLeaveEvent(self, e):
        QToolTip.hideText()
        super().hoverLeaveEvent(e)

    def mousePressEvent(self, e):
        InfoPage(self.info, self.basepage, self.geom.centroid)

        super().mousePressEvent(e)

class InfoPage(QWidget):
    def __init__(self, info: str, basepage, target_point):
        self.basepage = basepage
        self.map_widget = basepage.map_widget
        super().__init__(parent=self.basepage) 
        self.info = info
        
        self.target_point = target_point
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("background-color: white; border: 2px solid #888;")
        self.setFixedSize(300, 400)  # 크기 명시
        self.setWindowFlags(Qt.WindowType.WindowStaysOnTopHint | Qt.WindowType.FramelessWindowHint)

        layout = QVBoxLayout(self)
        base_path = os.path.expanduser("~/gui_project/gui_ubuntu/images")
        safe_name = self.info.replace(" ", "_")  # 띄어쓰기 안전 처리
        img_path = os.path.join(base_path, f"{safe_name}.png")
        
        if os.path.exists(img_path):
            image_label = QLabel()
            pixmap = QPixmap(img_path)
            image_label.setPixmap(pixmap.scaledToWidth(280))  # 너비 제한 (옵션)
            layout.addWidget(image_label, alignment=Qt.AlignmentFlag.AlignCenter)
        else:
            # 이미지 없을 경우 텍스트 대체
            layout.addWidget(QLabel(info), alignment=Qt.AlignmentFlag.AlignCenter)

        # 버튼 3개 구성
        btn_layout = QHBoxLayout()
        path_btn = QPushButton("길찾기")
        path_btn.setFixedHeight(60)
        path_btn.setFont(QFont("Pretendard", 14, QFont.Weight.Bold))
        path_btn.clicked.connect(self.start_path)
        
        robot_btn = QPushButton("로봇 안내")
        robot_btn.setFixedHeight(60)
        robot_btn.setFont(QFont("Pretendard", 14, QFont.Weight.Bold))
        robot_btn.clicked.connect(self.find_robot_guide)
        
        close_btn = QPushButton("닫기")
        close_btn.setFixedHeight(60)
        close_btn.setFont(QFont("Pretendard", 14, QFont.Weight.Bold))
        close_btn.clicked.connect(self.close)
        for b in (path_btn, robot_btn, close_btn): btn_layout.addWidget(b)
        layout.addLayout(btn_layout)

        # 위치 계산
        center = self.map_widget.mapToGlobal(self.map_widget.rect().center())
        self.move(center.x() - self.width() // 2, center.y() - self.height() // 2)

        self.show()
        self.raise_()

    def start_path(self):
        layer = getattr(self.map_widget, 'layers', {}).get('room_n_stair')
        if layer is not None:
            match = layer[layer['name'] == self.info]
            if not match.empty:
                centroid = match.geometry.iloc[0].centroid
                self.map_widget.start_floor = "1F"
                self.map_widget.start_pt = Point(start_x, start_y)
                self.map_widget.end_pt = centroid
                self.map_widget.draw_path()
        self.close()

    def find_robot_guide(self):
        self.basepage.clearLeft()
        rdp = RobotDestinationPage(self.basepage)
        self.basepage.left_layout.addWidget(rdp)
        rdp.set_destination(self.info)
        self.close()




class MapWidget(QWidget):
    def __init__(self, parent, floor_code: str = "1F"):
        print(f"[DEBUG] MapWidget parent = {parent} ({type(parent)})")

        super().__init__(parent)
        self.basepage = parent
        self.geojson_folder = os.path.expanduser("~/gui_project/gui_ubuntu/geojson")
        self.image_folder = os.path.expanduser("~/gui_project/gui_ubuntu/images")

        self.floor_code = floor_code

        self.view = QGraphicsView()
        self.scene = QGraphicsScene()
        self.view.setScene(self.scene)
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)

        layout = QVBoxLayout(self)
        layout.addWidget(self.view)

        image_path = os.path.join(self.image_folder, "fullmap_gui.pgm")
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            self.bg_pixmap_item = QGraphicsPixmapItem(pixmap)
            self.bg_pixmap_item.setZValue(-10)
            self.bg_pixmap_item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
            self.scene.addItem(self.bg_pixmap_item)
        else:
            print(f"[경고] 배경 이미지 없음: {image_path}")

        self.layers = load_layers(self.geojson_folder, floor_code=self.floor_code)
        self._init_viewport_size()
        self.draw_layers()
        self.path_union = unary_union(self.layers['path'].geometry)
        self.graph = build_graph(self.layers['path'])
        
        
        
        # 확대 비율 지정 (층별로 다르게도 가능)
        scale_factors = {
            "1F": 1.3,
            "2F": 1.3
        }
        scale = scale_factors.get(self.floor_code, 1.0)

        # fitInView 후에 적용해야 함
        self.view.setSceneRect(self.scene.itemsBoundingRect())
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        self.view.scale(scale, scale)


        self.view.setSceneRect(self.scene.itemsBoundingRect())
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def coord_to_point(self, x: float, y: float) -> QPointF:
        return QPointF(x, -y)

    def _init_viewport_size(self):
        vp = self.view.viewport().size()
        self.widget_w = vp.width()
        self.widget_h = vp.height()
        self.scale_x = self.widget_w / (MAX_X - MIN_X)
        self.scale_y = self.widget_h / (MAX_Y - MIN_Y)

    def draw_layers(self):
        # outline
        outline = self.layers.get('outline')
        if outline is not None and not outline.empty:
            pen = QPen(QColor(0,0,0), 0.05)
            for geom in outline.geometry:
                lines = geom.geoms if isinstance(geom, MultiLineString) else [geom]
                for line in lines:
                    pts = list(line.coords)
                    for (x1,y1),(x2,y2) in zip(pts[:-1], pts[1:]):
                        p1 = self.coord_to_point(x1,y1)
                        p2 = self.coord_to_point(x2,y2)
                        li = QGraphicsLineItem(p1.x(),p1.y(),p2.x(),p2.y())
                        li.setPen(pen)
                        self.scene.addItem(li)

        # doors
        doors = self.layers.get('doors')
        if doors is not None and not doors.empty:
            pen = QPen(QColor(0,0,0), 0.05)
            brush = QBrush(QColor(236, 230, 204, 160))
            for r in doors.itertuples():
                info = getattr(r,'name',f'Door {r.Index}')
                geom = r.geometry
                if isinstance(geom,(Polygon,MultiPolygon)):
                    polys = geom.geoms if isinstance(geom,MultiPolygon) else [geom]
                    for poly in polys:
                        pts = [self.coord_to_point(x,y) for x,y in poly.exterior.coords]
                        it = InfoPolygonItem(QPolygonF(pts), info, self.basepage, self, poly)


                        it.setPen(pen); it.setBrush(brush)
                        self.scene.addItem(it)

        # rooms
        rooms = self.layers.get('room_n_stair')
        if rooms is not None and not rooms.empty:
            pen = QPen(QColor(0,0,0), 0.05)
            brush = QBrush(QColor(255,250,200,220))
            for r in rooms.itertuples():
                info = getattr(r,'name',f'Room {r.Index}')
                polys = r.geometry.geoms if isinstance(r.geometry,MultiPolygon) else [r.geometry]
                for poly in polys:
                    pts = [self.coord_to_point(x,y) for x,y in poly.exterior.coords]
                    it = InfoPolygonItem(QPolygonF(pts), info, self.basepage, self, poly)

                    it.setPen(pen); it.setBrush(brush)
                    self.scene.addItem(it)

    def draw_path(self):
    
        if not hasattr(self, 'start_pt') or not hasattr(self, 'end_pt'):
            print("[오류] draw_path() 중단: start_pt 또는 end_pt 미설정")
            return
        if self.start_pt is None or self.end_pt is None:
            print("[오류] draw_path() 중단: start_pt 또는 end_pt 값이 None임")
            return
        # 기존 경로 제거
        if hasattr(self, 'path_item') and self.path_item is not None:
            self.scene.removeItem(self.path_item)
            self.path_item = None

        if not hasattr(self, 'start_pt') or not hasattr(self, 'end_pt'):
            print("[오류] start_pt 또는 end_pt가 설정되지 않았습니다.")
            return

        # 다층 경로 처리: 시작 층과 현재 층이 다르면
        if hasattr(self, 'start_floor') and self.start_floor != self.floor_code:
            if hasattr(self, 'set_multi_floor_path'):
                self.set_multi_floor_path(self.start_pt, self.end_pt, self.floor_code)
            else:
                print("[오류] set_multi_floor_path 함수가 정의되지 않았습니다.")
            return

        # 단일 층 경로 처리
        path_layer = self.layers.get('path')
        if path_layer is None or path_layer.empty:
            print("[오류] 경로 레이어가 비어 있습니다.")
            return

        self.path_union = unary_union(path_layer.geometry)
        self.graph = build_graph(path_layer)

        s0 = nearest_points(self.start_pt, self.path_union)[1]
        e0 = nearest_points(self.end_pt, self.path_union)[1]
        G2 = self.graph.copy()
        sn = insert_projection(G2, s0)
        en = insert_projection(G2, e0)
        if sn is None or en is None:
            print("[경고] 투영 실패로 경로를 그릴 수 없습니다.")
            return

        try:
            path = nx.shortest_path(G2, sn, en, weight='weight')
        except nx.NetworkXNoPath:
            print("[경고] 경로를 찾을 수 없습니다.")
            return

        coords = [s0.coords[0]] + path + [e0.coords[0]]
        line = LineString(coords)

        painter = QPainterPath()
        painter.moveTo(self.coord_to_point(self.start_pt.x, self.start_pt.y))
        painter.lineTo(self.coord_to_point(s0.x, s0.y))

        for i, (x, y) in enumerate(line.coords):
            pt = self.coord_to_point(x, y)
            if i == 0:
                painter.moveTo(pt)
            else:
                painter.lineTo(pt)

        painter.moveTo(self.coord_to_point(e0.x, e0.y))
        painter.lineTo(self.coord_to_point(self.end_pt.x, self.end_pt.y))

        path_item = QGraphicsPathItem(painter)
        path_item.setPen(QPen(QColor(255, 0, 0), 0.2))
        path_item.setZValue(6)
        self.scene.addItem(path_item)
        self.path_item = path_item

        #self.view.setSceneRect(self.scene.itemsBoundingRect())
        #self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        print(f"[✅ 완료] {self.floor_code}층 경로 시각화 완료")


        
    # 계단 중 가장 가까운 것 반환
    def find_nearest_stair(self, pt):
        stairs = self.layers.get("stair")
        if stairs is None or stairs.empty:
            return None
        dists = stairs.geometry.apply(lambda g: pt.distance(g))
        nearest_idx = dists.idxmin()
        return stairs.loc[nearest_idx, "geometry"]

    # 1층에서 계단까지 경로 → 딜레이 후 2층 이동
    def set_multi_floor_path(self, start_point, end_point, end_floor):
        print(f"[다층 경로] 1층 → {end_floor}")

        self.floor_code = "1F"
        self.layers = load_layers(self.geojson_folder, floor_code="1F")
        self.path_union = unary_union(self.layers['path'].geometry)
        self.graph = build_graph(self.layers['path'])

        self.start_pt = start_point
        stair = self.find_nearest_stair(start_point)
        if stair is None or stair.is_empty:
            print("[오류] 1층 계단을 찾을 수 없습니다.")
            return

        self.end_pt = stair.centroid
        self._clear_previous_path()
        self._draw_segmented_path()
        

        QTimer.singleShot(1000, lambda: self._draw_second_floor_path(end_point, end_floor))



    # 2층으로 전환 후 계단 → 목적지 경로
    def _draw_second_floor_path(self, end_point, end_floor):
        print(f"[다층 경로 이어서] {end_floor}층 전환")

        self.floor_code = end_floor
        self.layers = load_layers(self.geojson_folder, floor_code=end_floor)

        if self.layers['path'].empty:
            print(f"[오류] {end_floor}층의 path 레이어가 비어 있습니다.")
            return

        self.path_union = unary_union(self.layers['path'].geometry)
        self.graph = build_graph(self.layers['path'])

        stair = self.find_nearest_stair(end_point)
        if stair is None or stair.is_empty:
            print("[오류] 2층 계단을 찾을 수 없습니다.")
            return

        stair_pt = stair.centroid
        self.start_pt = stair_pt
        self.end_pt = end_point
        self._clear_previous_path()
        self._draw_segmented_path()

        #self.view.setSceneRect(self.scene.itemsBoundingRect())
        #self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        #self.view.scale(1.5,1.5)
        print(f"[✅ 완료] {end_floor}층 경로 시각화 완료")
        
        # 2층 경로 시각화 끝에 추가
        parent = self.parent()
        while parent and not hasattr(parent, "floor_label"):
            parent = parent.parent()

        if parent:
            parent.selected_floor = end_floor
            parent.floor_label.setText(end_floor)
            for fl, btn in parent.floor_buttons.items():
                is_selected = (fl == end_floor)
                btn.setChecked(is_selected)
                btn.setStyleSheet(parent.getFloorButtonStyle(is_selected))

        
        
        
    def _clear_previous_path(self):
        if hasattr(self, 'path_item') and self.path_item is not None:
            self.scene.removeItem(self.path_item)
            self.path_item = None

    def _draw_segmented_path(self):
        s0 = nearest_points(self.start_pt, self.path_union)[1]
        e0 = nearest_points(self.end_pt, self.path_union)[1]
        G2 = self.graph.copy()
        sn = insert_projection(G2, s0)
        en = insert_projection(G2, e0)
        if sn is None or en is None:
            print("[경고] 투영 실패")
            return

        try:
            path = nx.shortest_path(G2, sn, en, weight='weight')
        except nx.NetworkXNoPath:
            print("[경고] 경로 없음")
            return
    
        coords = [s0.coords[0]] + path + [e0.coords[0]]
        line = LineString(coords)

        painter = QPainterPath()
        painter.moveTo(self.coord_to_point(self.start_pt.x, self.start_pt.y))
        painter.lineTo(self.coord_to_point(s0.x, s0.y))
        for i, (x, y) in enumerate(line.coords):
            pt = self.coord_to_point(x, y)
            painter.lineTo(pt)
        painter.moveTo(self.coord_to_point(e0.x, e0.y))
        painter.lineTo(self.coord_to_point(self.end_pt.x, self.end_pt.y))

        path_item = QGraphicsPathItem(painter)
        path_item.setPen(QPen(QColor(255, 0, 0), 0.2))
        path_item.setZValue(6)
        self.scene.addItem(path_item)
        self.path_item = path_item
        
    def resizeEvent(self, event):
        super().resizeEvent(event)
        #if hasattr(self, 'scene') and hasattr(self, 'view'):
            #self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)





#if __name__ == "__main__":
#    import sys
#    from PyQt6.QtWidgets import QApplication

#    base = os.path.dirname(os.path.abspath(__file__))
#    geojson = os.path.join(base, "geojson")
#    images = os.path.join(base, "gui_images")

#    app = QApplication(sys.argv)
#    w = MapWidget(floor_code="1F")
#    w.showMaximized()
#    sys.exit(app.exec())
