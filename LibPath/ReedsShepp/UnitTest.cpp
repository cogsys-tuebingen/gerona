/*
 * UnitTest.cpp
 *
 *  Created on: Sep 30, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include <iostream>
#include <sstream>
#include <vector>

#include "Curve.h"
#include "CurveGenerator.h"
#include "CurveRenderer.h"


void test_all_patterns()
{
  ReedsShepp::CurveGenerator generator;

  std::vector<std::string> patterns;

  patterns.push_back("LSL");
  patterns.push_back("LSR");
  patterns.push_back("RSL");
  patterns.push_back("RSR");

  patterns.push_back("|LSL");
  patterns.push_back("|LSR");
  patterns.push_back("|RSL");
  patterns.push_back("|RSR");

  patterns.push_back("|L|SL");
  patterns.push_back("|L|SR");
  patterns.push_back("|R|SL");
  patterns.push_back("|R|SR");

  patterns.push_back("LS|L");
  patterns.push_back("LS|R");
  patterns.push_back("RS|L");
  patterns.push_back("RS|R");

  patterns.push_back("L|SL");
  patterns.push_back("L|SR");
  patterns.push_back("R|SL");
  patterns.push_back("R|SR");

  patterns.push_back("L|S|L");
  patterns.push_back("L|S|R");
  patterns.push_back("R|S|L");
  patterns.push_back("R|S|R");

  patterns.push_back("|LS|L");
  patterns.push_back("|LS|R");
  patterns.push_back("|RS|L");
  patterns.push_back("|RS|R");

  patterns.push_back("|L|S|L");
  patterns.push_back("|L|S|R");
  patterns.push_back("|R|S|L");
  patterns.push_back("|R|S|R");

  patterns.push_back("LRL");
  patterns.push_back("RLR");

  // C|C|C
  patterns.push_back("L|R|L");
  patterns.push_back("R|L|R");
  patterns.push_back("|L|R|L");
  patterns.push_back("|R|L|R");

  // CC|C
  patterns.push_back("LR|L");
  patterns.push_back("RL|R");
  patterns.push_back("|LR|L");
  patterns.push_back("|RL|R");

  // C|CC
  patterns.push_back("L|RL");
  patterns.push_back("R|LR");
  patterns.push_back("|L|RL");
  patterns.push_back("|R|LR");

  patterns.push_back("LR(b)|L(b)R");
  patterns.push_back("|LR(b)|L(b)R");
  patterns.push_back("RL(b)|R(b)L");
  patterns.push_back("|RL(b)|R(b)L");

  patterns.push_back("L|R(b)L(b)|R");
  patterns.push_back("R|L(b)R(b)|L");
  patterns.push_back("|L|R(b)L(b)|R");
  patterns.push_back("|R|L(b)R(b)|L");

  std::vector<std::string>::iterator it;
  std::stringstream ss;
  for(it = patterns.begin(); it != patterns.end(); ++it){
    ss.clear(); ss.str("");
    generator.parse(*it, ss);
    if(ss.str().length() > 0){
      std::cout << std::endl << "TEST FAILED: pattern must be valid" << std::endl;
      std::cout << ss.str() << std::endl;
    }
  }
}

void test_errors()
{
  ReedsShepp::CurveGenerator generator;
  std::stringstream ss;

  ss.clear(); ss.str("");
  generator.parse("RPLR", ss);
  if(ss.str().length() == 0){
    std::cout << std::endl << "TEST FAILED: error: illegal char" << std::endl;
  }

  ss.clear(); ss.str("");
  generator.parse("R(LR", ss);
  if(ss.str().length() == 0){
    ss << std::endl << "TEST FAILED: error: not closed nesting" << std::endl;
  }

  ss.clear(); ss.str("");
  generator.parse("R((L))R", ss);
  if(ss.str().length() == 0){
    ss << std::endl << "TEST FAILED: error: too many levels" << std::endl;
  }

  ss.clear(); ss.str("");
  generator.parse("R)LR", ss);
  if(ss.str().length() == 0){
    ss << std::endl << "TEST FAILED: error: not opened nesting" << std::endl;
  }
}

int main(int argc, char *argv[]) {
  //////// CSC
  test_all_patterns();

  test_errors();

  std::cout << "done." << std::endl;
}
