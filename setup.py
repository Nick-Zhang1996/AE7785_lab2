from setuptools import setup

package_name = 'ZhiyuanZhang_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nick',
    maintainer_email='nickzhang@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate_robot = ZhiyuanZhang_object_follower.rotate_robot:main',
            'find_object = ZhiyuanZhang_object_follower.find_object:main',
            'send_test_img = ZhiyuanZhang_object_follower.send_test_img:main',
            'view_debug_img = ZhiyuanZhang_object_follower.view_debug_img:main'
        ],
    },
)
