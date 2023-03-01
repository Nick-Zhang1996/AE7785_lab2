from setuptools import setup

package_name = 'ZhiyuanZhang_navigate_to_goal'

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
            'get_object_range = ZhiyuanZhang_navigate_to_goal.getObjectRange:main',
            'goto_goal = ZhiyuanZhang_navigate_to_goal.goto_goal:main',
        ],
    },
)
