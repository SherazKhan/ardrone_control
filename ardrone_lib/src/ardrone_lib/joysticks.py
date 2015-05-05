#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
In this module there are definitions for the mapping
of PS3 and PS2 joysticks to the Joy msg runned by
rosrun joy joy_node.
"""
PS3 = {'buttons':['select',
                  'L3',
                  'R3',
                  'start',
                  'top_arrow',
                  'right_arrow',
                  'bottom_arrow',
                  'left_arrow',
                  'L2',
                  'R2',
                  'L1',
                  'R1',
                  'triangle',
                  'circle',
                  'cross',
                  'square',
                  'analog'],
       'axes': {'L3_LR': 0,
                'L3_UD': 1,
                'R3_LR': 2,
                'R3_UD': 3}
      }

PS2 = {'buttons':['triangle',
                  'circle',
                  'cross',
                  'square',
                  'L1',
                  'R1',
                  'L2',
                  'R2',
                  'select',
                  'start',
                  'L3',
                  'R3',
                  'analog'],
       'axes': {'L3_LR': 0,
                'L3_UD': 1,
                'R3_LR': 3,
                'R3_UD': 4,
                'DIR_LR': 6,
                'DIR_UD': 7}
      }
