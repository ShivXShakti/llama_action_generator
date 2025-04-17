from setuptools import find_packages, setup

package_name = 'llama_action_generator'

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
    maintainer='cstar',
    maintainer_email='kuldeeplakhansons@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat1 = llama_action_generator.chat1:main',
            'prompt = llama_action_generator.prompt:main',
            'llama_sub = llama_action_generator.llama_sub:main',
        ],
    },
)
