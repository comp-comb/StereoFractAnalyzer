from setuptools import setup, find_packages

setup(
    name="StereoFractAnalyzer",
    version='0.1.7',
    author='Khaled Mosharraf Mukut',
    author_email='kmmukut@gmail.com',
    description='The StereoFractAnalyzer class in this package is designed for fractal dimension analysis of both 3D meshes from STL files and 2D images, incorporating functionalities for geometric transformations, visualization, and fractal dimension calculation using the box counting method. It leverages libraries like open3d, numpy, matplotlib, and PIL for processing and supports command-line arguments for user convenience.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/kmmukut/StereoFractAnalyzer.git',
    license='MIT',
    entry_points={
    'console_scripts': ['StereoFractAnalyzer = StereoFractAnalyzer:main'],
    },
    packages=find_packages(),
    install_requires=[
        'open3d',
        'numpy',
        'matplotlib',
        'Pillow',
        'argparse'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)
