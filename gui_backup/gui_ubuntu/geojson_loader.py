import os
import glob
import geopandas as gpd

# ▶ GeoJSON 파일들이 들어있는 폴더 경로
geojson_dir = r"C:\Users\USER\OneDrive\바탕 화면\GUI\map_tyr_1"

# ▶ 폴더 내의 .geojson 파일들을 모두 찾기 위한 패턴
pattern = os.path.join(geojson_dir, "*.geojson")

# ▶ glob 으로 모든 GeoJSON 파일 경로 리스트 생성
geojson_files = glob.glob(pattern)

# ▶ 파일명(key) → GeoDataFrame(value) 형태로 저장할 딕셔너리
geo_dfs = {}

for filepath in geojson_files:
    # 파일 이름만 추출 (확장자 제외)
    name = os.path.splitext(os.path.basename(filepath))[0]
    # GeoJSON 파일을 읽어서 GeoDataFrame으로 변환
    gdf = gpd.read_file(filepath)
    # 필요하다면 중심점 컬럼 추가
    gdf['centroid'] = gdf.geometry.centroid
    # 딕셔너리에 저장
    geo_dfs[name] = gdf
    print(f"Loaded '{name}' ({len(gdf)} features)")

# ▶ 이제 geo_dfs 딕셔너리로 모든 GeoJSON 데이터에 접근 가능합니다.
# 예시) base.geojson 데이터프레임은 geo_dfs['base'] 로 불러오기
