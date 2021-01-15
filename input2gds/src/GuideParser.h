#ifndef GUIDEPARSER_H
#define GUIDEPARSER_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <fstream>
#include <deque>
#include <fstream>
#include <utility>

using namespace std;
class GuideParser
{
  public:
    GuideParser();


    struct rectangle
    {
        unsigned int lbx, lby, rtx, rty, layer;

        rectangle(unsigned int _lbx, unsigned int _lby, unsigned int _rtx, unsigned int _rty, unsigned int _layer)
        {
            lbx = _lbx;
            lby = _lby;
            rtx = _rtx;
            rty = _rty;
            layer = _layer;
        }
    };

    vector<vector<rectangle> > net_guide;
    vector<pair<int, int>> &count_height_base;
    vector<pair<int, int>> &count_width_base;
    vector<pair<int, int>> &count_height;
    vector<pair<int, int>> &count_width;
    
    void GuideCount();
    void ISPD2018_guide_fileInput(istream &file);
    
    void GuideCount()
    {
        // important parameter
        int DieArea[2] = {390800, 383040};

        vector<int> height_nonuniform, width_nonuniform;
        int max_candidate = 0;
        int max_count = 0;
        for (int index3 = 0; index3 < count_height.size(); index3++)
        {
            if (count_height.at(index3).second > max_count)
            {
                max_count = count_height.at(index3).second;
                max_candidate = count_height.at(index3).first;
            }
        }

        count_height_base.push_back(make_pair(max_candidate, 0));

        for (int index3 = 0; index3 < count_height.size(); index3++)
        {
            int now_number = count_height.at(index3).first % max_candidate;

            if (!count_height_base.empty())
            {
                bool found_same = false;
                for (int index4 = 0; index4 < count_height_base.size(); index4++)
                {
                    if (now_number % count_height_base.at(index4).first == 0)
                    {
                        if (count_height_base.at(index4).first != max_candidate)
                        {
                            // catch these cases
                            height_nonuniform.push_back(count_height.at(index3).first);
                        }

                        count_height_base.at(index4).second++;
                        found_same = true;
                        break;
                    }
                }
                if (!found_same)
                {
                    count_height_base.push_back(make_pair(now_number, 1));
                }
            }
            else
            {

            } // if
        }

        // width
        max_count = 0;
        for (int index3 = 0; index3 < count_width.size(); index3++)
        {
            if (count_width.at(index3).second > max_count)
            {
                max_count = count_width.at(index3).second;
                max_candidate = count_width.at(index3).first;
            }
        }

        count_width_base.push_back(make_pair(max_candidate, 0));

        for (int index3 = 0; index3 < count_width.size(); index3++)
        {
            int now_number = count_width.at(index3).first % max_candidate;

            if (!count_width_base.empty())
            {
                bool found_same = false;
                for (int index4 = 0; index4 < count_width_base.size(); index4++)
                {
                    if (now_number % count_width_base.at(index4).first == 0)
                    {
                        if (count_height_base.at(index4).first != max_candidate)
                        {
                            // catch these cases
                            width_nonuniform.push_back(count_height.at(index3).first);
                        }

                        count_width_base.at(index4).second++;
                        found_same = true;
                        break;
                    }
                }
                if (!found_same)
                {
                    count_width_base.push_back(make_pair(now_number, 1));
                }
            }
            else
            {

            } // if
        }
        //

        cout << "Kind of height count : " << count_height_base.size() << "\n";
        for (int index3 = 0; index3 < count_height_base.size(); index3++)
        {
            cout << "(" << count_height_base.at(index3).first << ") : " << count_height_base.at(index3).second << endl;
        }
        cout << "Kind of width count : " << count_width_base.size() << "\n";
        for (int index3 = 0; index3 < count_width_base.size(); index3++)
        {
            cout << "(" << count_width_base.at(index3).first << ") : " << count_width_base.at(index3).second << endl;
        }

        cout << "#height nonuniform list :\n";
        for (auto &index : height_nonuniform)
        {
            cout << index << endl;
        }
    }// GuideCount()

    void ISPD2018_guide_fileInput(istream &file)
    {
        vector<rectangle> temp_rect_list;

        string line;
        while (!file.eof())
        {
            getline(file, line); // net name
            vector<rectangle> temp_guide;

            if (line.size() > 0 && line.at(0) == '(')
            {
                string line_guide;
                getline(file, line_guide); // (
                while (line_guide.size() > 0 && line_guide.at(0) != ')')
                {
                    if (line_guide.find(" ") != string::npos)
                    {
                        int M_pos = line_guide.find(' ');

                        int index_start_coor;
                        unsigned int _x[2], _y[2], layer_number;

                        string layer_num_s;
                        layer_num_s.clear();
                        for (int ind = 0; ind < 4; ind++)
                        {
                            for (index_start_coor = 0; line_guide.at(index_start_coor) != ' ';)
                            {
                                layer_num_s.push_back(line_guide.at(index_start_coor));
                                line_guide.erase(line_guide.begin());
                            }

                            line_guide.erase(line_guide.begin());

                            if (ind == 0)
                                _x[0] = atoi(layer_num_s.c_str());
                            else if (ind == 1)
                                _x[1] = atoi(layer_num_s.c_str());
                            else if (ind == 2)
                                _y[0] = atoi(layer_num_s.c_str());
                            else if (ind == 3)
                                _y[1] = atoi(layer_num_s.c_str());

                            layer_num_s.clear();
                        } // for

                        layer_num_s.push_back(line_guide.at(line_guide.size() - 1));
                        layer_number = atoi(layer_num_s.c_str());

                        int c_height, c_width;
                        c_width = _y[0] - _x[0];
                        c_height = _y[1] - _x[1];

                        if (!count_height.empty())
                        {
                            bool found_same = false;
                            for (int index3 = 0; index3 < count_height.size(); index3++)
                            {
                                if (c_height % count_height.at(index3).first == 0)
                                {
                                    count_height.at(index3).second++;
                                    found_same = true;
                                    break;
                                }
                            }
                            if (!found_same)
                            {
                                count_height.push_back(make_pair(c_height, 1));
                            }
                        }
                        else
                        {
                            count_height.push_back(make_pair(c_height, 1));
                        } // if

                        if (!count_width.empty())
                        {
                            bool found_same = false;
                            for (int index3 = 0; index3 < count_width.size(); index3++)
                            {
                                if (c_width % count_width.at(index3).first == 0)
                                {
                                    count_width.at(index3).second++;
                                    found_same = true;
                                    break;
                                }
                            }
                            if (!found_same)
                            {
                                count_width.push_back(make_pair(c_width, 1));
                            }
                        }
                        else
                        {
                            count_width.push_back(make_pair(c_width, 1));
                        } // if

                        cout << ">> " << _x[0] << " " << _x[1] << " " << _y[0] << " " << _y[1] << " " << layer_number << endl;
                        rectangle temp_rect = rectangle(_x[0], _x[1], _y[0], _y[1], layer_number);
                        temp_guide.push_back(temp_rect);
                    }

                    getline(file, line_guide);

                } // while

                net_guide.push_back(temp_guide);

            } // if

            temp_guide.clear();
        }
        return;
    } // read input()
    
};

//


#endif