from pathlib import Path
import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

requirement_path = Path(__file__).parent / "requirements.txt"
install_requires = []
if requirement_path.exists():
    with open(requirement_path) as f:
        install_requires = [line for line in map(str.lstrip, f.read().splitlines())
                            if len(line) > 0 and not line.startswith('#')]

setuptools.setup(
    name="dequa_graph",
    version="0.1",
    author="DeQua",
    author_email="info@dequa.it",
    description="Graph library used by DeQua",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="www.dequa.it",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.9',
    install_requires=install_requires,
)
