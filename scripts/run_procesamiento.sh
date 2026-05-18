#!/bin/bash

# ============================================================
# SCRIPT DE EJECUCIÓN PARA PROCESAMIENTO DE NUBE 3D
# Repositorio: MAPEO-3D-KINECT-V1-V2
# ============================================================

set -e

echo "========================================"
echo "PROCESAMIENTO DE NUBE 3D"
echo "========================================"

# Ir a la raíz del repositorio, aunque ejecutes el script desde otra carpeta
cd "$(dirname "$0")/.."

# Activar entorno virtual si existe
if [ -d ".venv" ]; then
    echo "Activando entorno virtual..."
    source .venv/bin/activate
fi

# Archivo de entrada
INPUT_PLY="data/input/nube_original.ply"

# Carpeta de salida
OUTPUT_DIR="results/experimento_01"

# Crear carpeta de salida si no existe
mkdir -p "$OUTPUT_DIR"

# Ejecutar procesamiento
python3 src/Procesar_nube_3d.py \
    --input "$INPUT_PLY" \
    --output-dir "$OUTPUT_DIR" \
    --target-points 100000 \
    --tolerance 10000 \
    --voxel-start 0.015 \
    --voxel-max 0.05 \
    --voxel-step 0.001 \
    --scale-cloud 1.0 \
    --random-state 42 \
    --export-dae

echo "========================================"
echo "PROCESO FINALIZADO"
echo "Resultados guardados en: $OUTPUT_DIR"
echo "========================================"
