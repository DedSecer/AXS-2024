from setuptools import setup,find_packages

setup(
    name='robot_tools',
    version='1.0',
    packages=find_packages(),
    package_dir={"": "."},
    install_requires=['numpy>=1.19.5','scipy','matplotlib'],
    author='GHz',
    author_email='ghz23@mails.tsinghua.edu.cn',
    description='Robot Tools.',
    url='https://gitlab.com/OpenGHz/airbot_play_vision_python.git',
    license='MIT'
)