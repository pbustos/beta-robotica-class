#!/usr/bin/env python3
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import os
os.chdir('/home/pbustos/robocomp/components/beta-robotica-class/mission_planner')

log = open('/tmp/convert_result.log', 'w')
log.write("Script iniciado\n")
log.flush()

if True:
    try:
        from mrpt.pymrpt import mrpt
        log.write("pymrpt importado correctamente\n")

        input_file = "mapa.gridmap"
        log.write(f"Cargando {input_file}...\n")

        # Abrir archivo comprimido
        fIn = mrpt.io.CFileGZInputStream(input_file)
        arch = mrpt.serialization.archiveFrom(fIn)

        # Deserializar el objeto
        gridmap = mrpt.maps.COccupancyGridMap2D()
        arch.ReadObject(gridmap)
        fIn.close()

        resolution = gridmap.getResolution()
        log.write(f"Mapa cargado: {gridmap.getSizeX()} x {gridmap.getSizeY()} celdas\n")
        log.write(f"Resolución: {resolution} m/celda ({resolution * 100:.2f} cm/pixel)\n")
        log.write(f"Tamaño real: {gridmap.getSizeX() * resolution:.2f} x {gridmap.getSizeY() * resolution:.2f} metros\n")
        log.write(f"Origen X: {gridmap.getXMin()}, Y: {gridmap.getYMin()}\n")

        # Guardar como imagen
        output_file = "mapa.png"
        gridmap.saveAsBitmapFile(output_file)
        log.write(f"Mapa guardado como: {output_file}\n")
        log.flush()

    except Exception as e:
        import traceback
        log.write(f"Error: {e}\n")
        log.write(traceback.format_exc())
        log.flush()
    finally:
        log.close()
