from setuptools import find_packages, setup

install_requires = ["fastapi", "uvicorn"]

setup(
    name="pf400_driver",
    version="0.1.0",
    packages=find_packages(),
    data_files=[],
    install_requires=install_requires,
    zip_safe=True,
    python_requires=">=3.8",
    maintainer="Doga Ozgulbas and Alan Wang",
    maintainer_email="dozgulbas@anl.gov",
    description="Driver for the PF400 robot arm",
    url="https://github.com/AD-SDL/pf400_module.git",
    license="MIT License",
    entry_points={
        "console_scripts": [
            "pf400_driver = pf400_driver.pf400_driver:main_null",
            "tcp_driver = pf400_driver.tcp_driver:main_null",
            "pf400_camera_driver =  pf400_driver.pf400_camera_driver:main_null",
        ]
    },
    classifiers=[
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX",
        "Operating System :: MacOS :: MacOS X",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
)
