import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="jknowrob",
    version="0.0.1",
    author="Sascha Jongebloed",
    author_email="sasjonge@uni-bremen.de",
    description="A Jupyter Kernel for SWI-Prolog.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/sasjonge/jupyer-knowrob.git",
    packages=setuptools.find_packages(),
    install_requires=[
        "pyswip",
        "ipykernel"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    entry_points={
        'console_scripts': ['jknowrobkernel=jknowrob.jupyter:main'],
    }
)
