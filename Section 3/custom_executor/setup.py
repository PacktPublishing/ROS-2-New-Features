from setuptools import setup

package_name = 'custom_executor'
# py_modules
custom_executor = 'custom_executor.custom_executor_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=['custom_executor'],
    py_modules=[
      custom_executor,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shane Loretz',
    author_email='sloretz@openrobotics.org',
    maintainer='Shane Loretz',
    maintainer_email='sloretz@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of creating and using exectors to run multiple nodes in rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_executor_example = {}:main'.format(custom_executor),
        ],
    },
)
