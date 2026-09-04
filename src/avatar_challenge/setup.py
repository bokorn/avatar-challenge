from setuptools import setup

package_name = 'avatar_challenge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['spatialmath-python'],
    zip_safe=True,
    author='Your Name',
    author_email='youremail@domain.com',
    description='Avatar Challenge Python package',
    entry_points={
        'console_scripts': [
            'draw_node = avatar_challenge.draw_node:main',
        ],
    },
)
