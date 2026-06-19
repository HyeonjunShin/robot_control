import mujoco
# URDF 파일을 읽어서 XML(MJCF) 구조로 저장합니다.
model = mujoco.MjModel.from_xml_path('m1013.urdf')
mujoco.mj_saveLastXML('m1013_mujoco.xml', model)
print("컴파일 완료: m1013_mujoco.xml 파일이 생성되었습니다.")
