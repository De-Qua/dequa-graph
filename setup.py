from pathlib import Path
import setuptools
from setuptools import Extension
import subprocess
import sys
import os

# --- Configurazione Estensione C++ ---

# Percorso del file sorgente C++
source_file = "src/dequa_get_dists.cc"

# Nome del modulo che Python importerà (deve corrispondere al nome del file .so senza estensione)
# Nel tuo caso, vuoi importare da "lib_dequadistance"
ext_name = "lib_dequadistance"

# Recupera i flag da pkg-config per graph-tool
# Nota: graph-tool-py3.9 è specifico per Python 3.9. 
# Se l'utente ha Python 3.10, potrebbe dover usare graph-tool-py3.10.
# Qui proviamo a usare il nome dinamico o quello fisso se funziona.
pkg_name = "graph-tool-py3.9" 
pkg_cflags = []
pkg_libs = []

try:
    # Esegui pkg-config
    result = subprocess.run(
        ["pkg-config", "--cflags", "--libs", pkg_name], 
        capture_output=True, 
        text=True, 
        check=False
    )
    
    if result.returncode == 0:
        # Split dei flag (es: -I... -L... -l...)
        pkg_cflags = result.stdout.split()
        pkg_libs = result.stdout.split()
    else:
        # Se fallisce, stampiamo un avviso ma non fermiamo l'installazione qui
        # (fallirà in fase di compilazione se graph-tool manca)
        print(f"Warning: pkg-config non ha trovato '{pkg_name}'. Verifica che graph-tool sia installato.", file=sys.stderr)

except FileNotFoundError:
    print("Error: pkg-config non trovato. È necessario per compilare graph-tool.", file=sys.stderr)

# Definizione dell'estensione
cpp_ext = Extension(
    name=ext_name,
    sources=[source_file],
    language="c++",
    extra_compile_args=[
        "-O3", 
        "-fopenmp", 
        "-Wfatal-errors", 
        "-std=gnu++17", 
        "-fPIC"
    ] + pkg_cflags,
    extra_link_args=pkg_libs,
)

# --- Configurazione setup.py classica ---
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
    version="1.0.0",
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
    ext_modules=[cpp_ext],  # <--- Qui dici a setuptools di compilare il C++
    include_package_data=True,
)