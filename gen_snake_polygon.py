#!/usr/bin/python

from __future__ import print_function
import argparse

ipe_format = False

class Point:
  def __init__(self, x, y):
    self.m_x = x
    self.m_y = y

  def __add__(self, other):
    return Point(self.m_x + other.m_x, self.m_y + other.m_y)

def print_point(p, fp, first=False):
  if not ipe_format:
    fp.write('{} {} '.format(p.m_x, p.m_y))
  else:
    if first:
      fp.write('{} {} m '.format(p.m_x, p.m_y))
    else:
      fp.write('{} {} l '.format(p.m_x, p.m_y))

def generate(p, length, size, left_to_right, turns, fp):
  if (1 == turns):
    if left_to_right:
      p.m_x += length
      print_point(p, fp)
      p.m_y += 2 * size
      print_point(p, fp)
      p.m_x -= 2 * size
      print_point(p, fp)
      p.m_y -= size
      print_point(p, fp)
      p.m_x += size
      return

    # right to left
    p.m_x += size
    p.m_x -= length
    print_point(p, fp)
    p.m_y += size
    print_point(p, fp)
    p.m_x -= 2 * size
    print_point(p, fp)
    p.m_y -= 2 * size
    print_point(p, fp)
    return

  if left_to_right:
    p.m_x += length
    print_point(p, fp)
    p.m_y += 3 * size
    print_point(p, fp)
  else:
    p.m_x -= length
    print_point(p, fp)
    p.m_y += size
    print_point(p, fp)

  generate(p, length, size, not left_to_right, turns-1, fp)
  if left_to_right:
    p.m_x += length
    print_point(p, fp)
    p.m_y -= size
    print_point(p, fp)
  else:
    p.m_x -= length
    print_point(p, fp)
    p.m_y -= 3 * size
    print_point(p, fp)

# Main function
if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Generate a snake-like polygon.')
  parser.add_argument('obstaclesFileName', help='file to save the obstacles to')

  parser.add_argument('--unitSize', type=float, default=1.0, help='Unit size')
  parser.add_argument('--turns', type=int, default=4, help='Number of turns')
  parser.add_argument('--length', type=int, default=4, help='Leg length')

  parser.add_argument('--ipeFormat', dest='ipeFormat', action='store_true')
  parser.add_argument('--noIpeFormat', dest='ipeFormat', action='store_false')
  parser.set_defaults(ipeFormat=ipe_format)

  args = parser.parse_args()
  size = args.unitSize
  length = args.length
  turns = args.turns
  ipe_format = args.ipeFormat
  obstaclesFileName = args.obstaclesFileName

  if (turns < 1):
    parser.error('The number of turns ({}) must be greater then 0!'.format(turns))
  if (length < 4 * size):
    parser.error('The length ({}) must be at least {} (4 x unit-size)!'.format(length, 4 * size))

  p = Point(0.0, 0.0)

  fp = open(obstaclesFileName,'w')
  n = 2 + 4 * turns
  fp.write('{} \n'.format(n))
  print_point(p, fp, first=True)

  p.m_x += size;
  generate(p, length, size, True, turns, fp)
  p.m_x -= length;
  print_point(p, fp)
  fp.write('\n0 \n'.format(n))
  fp.close
  
  exit()