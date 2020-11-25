//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H

#include <QGraphicsItem>


template<typename HMIN, HMIN hmin_, typename WIDTH, WIDTH width_, typename TILE, TILE tile_>
class Grid
{
    int hmin, width, tile;
    public:
        Grid()
        {
            hmin = hmin_; width = width_; tile = tile_;
            array.resize((int)(width/tile));
            for (auto &row : array)
                row.resize((int)(width/tile));
            int k=0;
            for (int i = hmin; i < width/2; i += tile, k++)
            {
                int l=0;
                for (int j = hmin; j < width/2; j += tile, l++)
                {
                    array[k][l] = Value{false, nullptr, i, j};
                }
            }
        };

        struct Value
        {
            bool occupied = false;
            QGraphicsRectItem * paint_cell = nullptr;
            int cx, cy;
            int dist = 0; //dist vecinos
        };

        std::vector<std::vector<Value>> array;

        void create_graphic_items(QGraphicsScene &scene)
        {
            for (auto &row : array)
                for (auto &elem : row)
                {
                    elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("DarkGreen")), QBrush(QColor("LightGreen")));
                    elem.paint_cell->setPos(elem.cx, elem.cy);
                }
        }

        /**
         * modificamos en funcion de v la coordenada x,z
         * @param x
         * @param z
         * @param v
         */
        void set_occupied(int x, int z)
        {
            auto [i, j] = transform(x,z);
            array[i][j].paint_cell->setBrush(QColor("Red"));
        }
        /**
         * devolvemos el valor de la coordenada x,z
         * @param x
         * @param z
         * @return
         */
        bool get_value(int x, int z)
        {
            auto [i, j] = transform(x,z);
            return  this->array[i][j];
        }

        std::tuple<int, int> transform(int x, int y)
        {
            int k = (1.f/tile)*x + (width/tile)/2;
            int l = (1.f/tile)*y + (width/tile)/2;
            return std::make_tuple(k,l);
        }
};


#endif //GOTOXY_GRID_H
