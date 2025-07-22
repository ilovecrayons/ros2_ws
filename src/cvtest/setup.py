from setuptools import find_packages, setup

package_name = 'cvtest'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huang',
    maintainer_email='huangfeiyang2008@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cv_node = cvtest.cv_node:main",
            "linreg_test = cvtest.linreg_test:main",
        ],
    },
)
