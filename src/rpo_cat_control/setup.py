from setuptools import setup
setup(name='rpo_cat_control',version='0.1.0',packages=['rpo_cat_control'],
 data_files=[('share/ament_index/resource_index/packages',['resource/rpo_cat_control']),('share/rpo_cat_control',['package.xml'])],
 install_requires=['setuptools'],zip_safe=True,
 entry_points={'console_scripts':['motion_player = rpo_cat_control.player:main','mock_robot = rpo_cat_control.mock_robot:main','supported_sdk_bridge = rpo_cat_control.sdk_bridge:main']})
