from setuptools import setup

package_name = 'opencv_test_py'

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
    maintainer='bcalli',
    maintainer_email='bcalli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'image_processor = opencv_test_py.test1:main',
             'canny = opencv_test_py.canny_detection:main',
             'harris = opencv_test_py.harris_detection:main',
             'hough = opencv_test_py.hough_circles:main',
        ],
    },
)
