from setuptools import setup

package_name = 'packt_rclpy_demo'
# py modules
member_function = 'packt_rclpy_demo.publisher_member_function'
listener = 'packt_rclpy_demo.subscriber_member_function'

setup(
    name=package_name,
    version='0.0.0',
    packages=['packt_rclpy_demo'],
    py_modules=[
      member_function,
      listener
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='packt',
    author_email="packt@todo.todo",
    maintainer='packt',
    maintainer_email="packt@todo.todo",
    keywords=['ROS2', 'packt', 'rclpy', 'publisher', 'subscriber', 'demo'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'talker = {}:main'.format(member_function),
          'listener = {}:main'.format(listener)
        ],
    },
)
