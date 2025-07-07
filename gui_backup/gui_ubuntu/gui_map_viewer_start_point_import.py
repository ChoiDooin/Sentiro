import sys
import os
import glob
import random
import pandas as pd

import geopandas as gpd
import networkx as nx
from shapely.geometry import Point, LineString, MultiLineString, Polygon, MultiPolygon
from shapely.ops import unary_union, nearest_points

from robot_point_importer import dummy_x, dummy_y

from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView, QGraphicsScene, QGraphicsPolygonItem, QGraphicsLineItem, QGraphicsEllipseItem, QGraphicsPathItem, QPushButton, QToolTip, QMessageBox, QDialog, QLabel, QStackedLayout
    
from PyQt6.QtGui import QPolygonF, QPainter, QPainterPath, QPen, QBrush, QColor, QPixmap, QCursor
from PyQt6.QtCore import QPointF, Qt, QTimer


def load_layers(folder: str, floor_code: str = "1F") -> dict:
    """
    각 레이어를 읽어와 dict로 반환.
    floor_code에 따라 shops_<floor_code>.geojson, path_<floor_code>.geojson을 사용.
    """
    layers = {}
    layers['base']    = gpd.read_file(os.path.join(folder, 'base_fullmap.geojson'))
    layers['outline'] = gpd.read_file(os.path.join(folder, 'outline_fullmap.geojson'))
    layers['doors']   = gpd.read_file(os.path.join(folder, 'doors_fullmap.geojson'))
    stair_elev = gpd.read_file(os.path.join(folder, 'stair_n_elevator_fullmap.geojson'))
    
    
    layers['stair'] = stair_elev[stair_elev['id'] == 0].reset_index(drop=True)      # 계단
    layers['elevator'] = stair_elev[stair_elev['id'] == 1].reset_index(drop=True)   # 엘리베이터


    # 층별 매장 + 계단 합치기
    shop_path = os.path.join(folder, f'shops_fullmap_{floor_code}.geojson')
    if os.path.exists(shop_path):
        shops = gpd.read_file(shop_path)
        layers['room_n_stair'] = pd.concat([shops, layers['stair'], layers['elevator']], ignore_index=True)
    else:
        layers['room_n_stair'] = pd.concat([layers['stair'], layers['elevator']], ignore_index=True)

    # 층별 path
    path_path = os.path.join(folder, f'path_fullmap_{floor_code}.geojson')
    if os.path.exists(path_path):
        layers['path'] = gpd.read_file(path_path)
    else:
        layers['path'] = gpd.GeoDataFrame(columns=['geometry'], geometry='geometry')

    return layers


def build_graph(path_gdf: gpd.GeoDataFrame) -> nx.Graph:
    G = nx.Graph()
    union = unary_union(path_gdf.geometry)
    lines = union.geoms if isinstance(union, MultiLineString) else [union]
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

class InfoPage(QWidget):
    """
    클릭/터치 시 화면 중앙에 작은 InfoPage 위젯 (경로찾기, 로봇 안내, 닫기 기능)
    """
    def __init__(self, info: str, image_path: str, map_widget, target_point=None):
        super().__init__(parent=map_widget)
        self.map_widget = map_widget
        self.target_point = target_point

        # 배경 스타일
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("background-color: white; border: 2px solid #888;")
        self.setFixedSize(300, 450)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # 정보 텍스트
        lbl = QLabel(info)
        lbl.setWordWrap(True)
        layout.addWidget(lbl)

        # 이미지가 있으면 표시
        if image_path and os.path.exists(image_path):
            pix = QPixmap(image_path).scaled(
                200, 200, Qt.AspectRatioMode.KeepAspectRatio
            )
            img_lbl = QLabel()
            img_lbl.setPixmap(pix)
            layout.addWidget(img_lbl)

        # 버튼 레이아웃
        btn_layout = QHBoxLayout()
        # 경로찾기 버튼
        path_btn = QPushButton("길찾기")
        path_btn.clicked.connect(self.find_path)
        btn_layout.addWidget(path_btn)
        # 로봇 안내 버튼
        robot_btn = QPushButton("로봇 안내")
        robot_btn.clicked.connect(self.find_robot_guide)
        btn_layout.addWidget(robot_btn)
        # 닫기 버튼
        close_btn = QPushButton("닫기")
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)
        layout.addLayout(btn_layout)

        # 화면 중앙 배치
        self.adjustSize()
        mw = self.map_widget
        x = (mw.width() - self.width()) // 2
        y = (mw.height() - self.height()) // 2
        self.move(x, y)
        self.show()

    def find_path(self):
        # 맵 위 경로 그리기
        self.map_widget.end_pt = self.target_point
        self.map_widget.draw_path()
        self.close()

    def find_robot_guide(self):
        from robot_destination_page import RobotDestinationPage
        # Basepage 찾아서 로봇 목적지 페이지로 이동
        parent = self.map_widget
        while parent and not hasattr(parent, 'clearLeft'):
            parent = parent.parent()
        if parent:
            parent.clearLeft()
            rdp = RobotDestinationPage(parent)
            parent.left_layout.addWidget(rdp)
            rdp.set_destination(self.info)
        self.close()

class InfoPage(QWidget):
    """
    클릭/터치 시 화면 중앙에 작은 InfoPage 위젯 (경로찾기, 로봇 안내, 닫기 기능)
    """
    def __init__(self, info: str, image_path: str, map_widget, target_point=None):
        super().__init__(parent=map_widget)
        self.map_widget = map_widget
        self.target_point = target_point
        self.info = info
        self.image_path = image_path

        # 배경 스타일
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet("background-color: white; border: 2px solid #888;")
        self.setFixedSize(300, 450)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        lbl = QLabel(info)
        lbl.setWordWrap(True)
        layout.addWidget(lbl)

        if image_path and os.path.exists(image_path):
            pix = QPixmap(image_path).scaled(200, 200, Qt.AspectRatioMode.KeepAspectRatio)
            img_lbl = QLabel()
            img_lbl.setPixmap(pix)
            layout.addWidget(img_lbl)

        btn_layout = QHBoxLayout()
        path_btn = QPushButton("길찾기")
        path_btn.clicked.connect(self.find_path)
        btn_layout.addWidget(path_btn)

        robot_btn = QPushButton("로봇 안내")
        robot_btn.clicked.connect(self.find_robot_guide)
        btn_layout.addWidget(robot_btn)

        close_btn = QPushButton("닫기")
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)

        layout.addLayout(btn_layout)

        # 화면 중앙
        self.adjustSize()
        mw = self.map_widget
        x = (mw.width() - self.width()) // 2
        y = (mw.height() - self.height()) // 2
        self.move(x, y)
        self.show()

    def find_path(self):
        # 맵 위 경로 그리기 (다층 경로 지원)
        if hasattr(self.map_widget, 'set_multi_floor_path') and self.map_widget.floor_code != self.map_widget.start_floor:
            self.map_widget.set_multi_floor_path(
                self.map_widget.start_pt,
                self.target_point,
                self.map_widget.floor_code
            )
        else:
            self.map_widget.end_pt = self.target_point
            self.map_widget.draw_path()
        self.close()

    def find_robot_guide(self):
        from robot_destination_page import RobotDestinationPage
        parent = self.map_widget
        while parent and not hasattr(parent, 'clearLeft'):
            parent = parent.parent()
        if parent:
            parent.clearLeft()
            rdp = RobotDestinationPage(parent)
            parent.left_layout.addWidget(rdp)
            rdp.set_destination(self.info)
        self.close()


class InfoPolygonItem(QGraphicsPolygonItem):
    def __init__(self, polygon: QPolygonF, info: str, image_path: str, map_widget, geom):
        super().__init__(polygon)
        self.map_widget = map_widget
        self.geom = geom
        self.info = info
        self.image_path = image_path
        self.setAcceptHoverEvents(True)

    def hoverEnterEvent(self, e):
        QToolTip.showText(e.screenPos(), str(self.info))
        super().hoverEnterEvent(e)

    def hoverLeaveEvent(self, e):
        QToolTip.hideText()
        super().hoverLeaveEvent(e)

    def mousePressEvent(self, e):
        InfoPage(self.info, self.image_path, self.map_widget, self.geom.centroid)
        super().mousePressEvent(e)


class InfoEllipseItem(QGraphicsEllipseItem):
    def __init__(self, x: float, y: float, w: float, h: float,
                 info: str, image_path: str, map_widget, geom):
        super().__init__(x, y, w, h)
        self.map_widget = map_widget
        self.geom = geom
        self.info = info
        self.image_path = image_path
        self.setAcceptHoverEvents(True)

    def hoverEnterEvent(self, e):
        QToolTip.showText(e.screenPos(), str(self.info))
        super().hoverEnterEvent(e)

    def hoverLeaveEvent(self, e):
        QToolTip.hideText()
        super().hoverLeaveEvent(e)

    def mousePressEvent(self, e):
        InfoPage(self.info, self.image_path, self.map_widget, self.geom)
        super().mousePressEvent(e)

class MapWidget(QWidget):
    def __init__(self, geojson_folder: str, image_folder: str, floor_code: str = "1F"):
        super().__init__()
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.FramelessWindowHint)
        self.geojson_folder = geojson_folder
        self.image_folder = image_folder
        self.floor_code = floor_code
        self.start_floor = "1F"

        self.layers = load_layers(self.geojson_folder, floor_code=self.floor_code)
        base_gdf = self.layers.get('base')
        rooms_gdf = self.layers.get('room_n_stair')
        self.base_union = unary_union(base_gdf.geometry) if base_gdf is not None else None
        self.rooms_union = unary_union(rooms_gdf.geometry) if rooms_gdf is not None else None
        self.walkway = (
            self.base_union.difference(self.rooms_union)
            if self.base_union and self.rooms_union else self.base_union
        )

        path_gdf = self.layers.get('path')
        if path_gdf is None or path_gdf.empty:
            self.graph = nx.Graph()
            self.path_union = None
        else:
            clipped = path_gdf.geometry.intersection(self.walkway) if self.walkway else path_gdf.geometry
            self.path_union = unary_union(clipped)
            tmp = path_gdf.copy()
            tmp['geometry'] = clipped
            self.graph = build_graph(tmp)
            if self.rooms_union:
                for u, v in list(self.graph.edges()):
                    if LineString([u, v]).intersects(self.rooms_union):
                        self.graph.remove_edge(u, v)

        minx, miny, maxx, maxy = self.walkway.bounds
        self.scale_x = (maxx - minx) / 30.0
        self.scale_y = (maxy - miny) / 30.0

        self.view = QGraphicsView()
        self.scene = QGraphicsScene()
        self.view.setScene(self.scene)
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)

        layout = QVBoxLayout(self)
        layout.addWidget(self.view)
        self.draw_layers()

        map_x = dummy_x * self.scale_x + minx
        map_y = maxy - dummy_y * self.scale_y
        self.start_pt = Point(map_x, map_y)
        self.path_item = None

        self.view.setSceneRect(self.scene.itemsBoundingRect())
        self.view.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)


    def coord_to_point(self, x: float, y: float) -> QPointF:
        return QPointF(x, -y)

    def draw_layers(self):
        # base polygon
        base = self.layers.get('base')
        if base is not None and not base.empty:
            pen = QPen(QColor(150,150,150), 0.05)
            brush = QBrush(QColor(200,200,200,100))
            for geom in base.geometry:
                polys = geom.geoms if isinstance(geom, MultiPolygon) else [geom]
                for poly in polys:
                    pts = [self.coord_to_point(x,y) for x,y in poly.exterior.coords]
                    item = QGraphicsPolygonItem(QPolygonF(pts))
                    item.setPen(pen)
                    item.setBrush(brush)
                    self.scene.addItem(item)

        # outline lines
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

        # doors (1층만 표시)
        if self.floor_code == "1F":
            doors = self.layers.get('doors')
            if doors is not None and not doors.empty:
                pen = QPen(QColor(0,0,0), 0.05)
                brush = QBrush(QColor(236, 230, 204, 160))
                for r in doors.itertuples():
                    info = getattr(r,'name',f'Door {r.Index}')
                    geom = r.geometry
                    items = []
                    if isinstance(geom,(Polygon,MultiPolygon)):
                        polys = geom.geoms if isinstance(geom,MultiPolygon) else [geom]
                        for poly in polys:
                            pts = [self.coord_to_point(x,y) for x,y in poly.exterior.coords]
                            items.append(InfoPolygonItem(QPolygonF(pts), info, None, self, poly))
                    else:
                        pts = geom.geoms if hasattr(geom,'geoms') else [geom]
                        for pt in pts:
                            qp = self.coord_to_point(pt.x,pt.y)
                            items.append(InfoEllipseItem(qp.x()-3, qp.y()-3, 6,6, info, None, self, pt))
                    for it in items:
                        it.setPen(pen)
                        it.setBrush(brush)
                        self.scene.addItem(it)


        # rooms (shops + stair)
        rooms = self.layers.get('room_n_stair')
        if rooms is not None and not rooms.empty:
            pen = QPen(QColor(0,0,0), 0.05)
            brush = QBrush(QColor(255, 250, 200, 180))
            for r in rooms.itertuples():
                info = getattr(r,'name',f'Room {r.Index}')
                img = os.path.join(self.image_folder, f"{info}.png")
                polys = r.geometry.geoms if isinstance(r.geometry,MultiPolygon) else [r.geometry]
                for poly in polys:
                    pts = [self.coord_to_point(x,y) for x,y in poly.exterior.coords]
                    it = InfoPolygonItem(QPolygonF(pts), info, img, self, poly)
                    it.setPen(pen)
                    it.setBrush(brush)
                    self.scene.addItem(it)

    def draw_path(self):
        if not hasattr(self,'start_pt') or not hasattr(self,'end_pt'):
            return
        if self.path_item:
            self.scene.removeItem(self.path_item)
            self.path_item = None

        s0 = nearest_points(self.start_pt, self.walkway)[1] if self.walkway else self.start_pt
        e0 = nearest_points(self.end_pt, self.walkway)[1] if self.walkway else self.end_pt
        sp = nearest_points(s0, self.path_union)[1]
        ep = nearest_points(e0, self.path_union)[1]

        G2 = self.graph.copy()
        sn = insert_projection(G2, sp)
        en = insert_projection(G2, ep)
        if sn is None or en is None:
            return
        try:
            path = nx.shortest_path(G2, sn, en, weight='weight')
        except nx.NetworkXNoPath:
            return

        coords = [(s0.x,s0.y),(sp.x,sp.y)] + path + [(ep.x,ep.y),(e0.x,e0.y)]
        full = LineString(coords)
        clip = full.intersection(self.walkway) if self.walkway else full

        if isinstance(clip,MultiLineString):
            segs = list(clip.geoms)
        elif isinstance(clip,LineString):
            segs = [clip]
        elif hasattr(clip,'geoms'):
            segs = [g for g in clip.geoms if isinstance(g,LineString)]
        else:
            segs = []

        painter = QPainterPath()
        for seg in segs:
            for i,(x,y) in enumerate(seg.coords):
                pt = self.coord_to_point(x,y)
                if i==0 and painter.elementCount()==0:
                    painter.moveTo(pt)
                else:
                    painter.lineTo(pt)

        self.path_item = QGraphicsPathItem(painter)
        self.path_item.setPen(QPen(QColor(255,0,0),0.1))
        self.path_item.setZValue(6)
        self.scene.addItem(self.path_item)

    def set_multi_floor_path(self, start_geom, end_geom, end_floor):
        # 1층 맵
        mw1 = MapWidget(self.geojson_folder, self.image_folder, floor_code="1F")
        stair1 = self._find_nearest_geom(start_geom, mw1.layers['stair'])
        stair1_geom = stair1.geometry
        mw1.start_pt, mw1.end_pt = start_geom, stair1_geom.centroid
        mw1.draw_path()

        # 도착층 맵
        mw2 = MapWidget(self.geojson_folder, self.image_folder, floor_code=end_floor)
        stair2 = self._find_matching_stair(stair1, mw2.layers['stair'])
        mw2.start_pt, mw2.end_pt = stair2.centroid, end_geom
        mw2.draw_path()

        # Basepage.map_container 스택에 추가·전환
        parent = self.parent()
        while parent is not None and not hasattr(parent, 'map_container'):
            parent = parent.parent()
        if parent is None:
            return
        stack = parent.map_container.layout()
        # 이전 내용 초기화
        while stack.count():
            w = stack.widget(0)
            stack.removeWidget(w)
            w.setParent(None)

        stack.addWidget(mw1)
        stack.addWidget(mw2)
        stack.setCurrentWidget(mw1)
        parent.floor_label.setText("1F")

        def _switch():
            stack.setCurrentWidget(mw2)
            parent.floor_label.setText(end_floor)
        QTimer.singleShot(3000, _switch)

    def _find_nearest_geom(self, target, layer):
        # layer 인자로 반드시 layers['stair']만 전달되도록 호출부를 점검하세요.
        idx = layer.geometry.distance(target).idxmin()
        return layer.loc[idx]



    def _find_matching_stair(self, stair1, layer2):
        name = getattr(stair1, 'name', None)
        if name and 'name' in layer2.columns:
            match = layer2[layer2['name'] == name]
            if not match.empty:
                return match.geometry.iloc[0]
        return layer2.geometry.iloc[0]

    def clear_path(self):
        """시작/끝점 초기화"""
        self.start_pt = None
        self.end_pt = None
        self.scene.update()


if __name__ == "__main__":
    base = os.path.dirname(os.path.abspath(__file__))
    geojson = os.path.join(base, "geojson")
    images = os.path.join(base, "gui_images")
    app = QApplication(sys.argv)
    w = MapWidget(geojson, images)
    w.showMaximized()
    sys.exit(app.exec())
