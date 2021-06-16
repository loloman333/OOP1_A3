//----------------------------------------------------------------------------------------------------------------------
// main.cpp
//
// Reads cli parameter and starts the game with or without bonus items
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "Game.hpp"
#include <iostream>
#include <string>

const std::string WRONG_ARGUMENTS = "Wrong arguments!";
const std::string BONUS_ARGUEMT = "-bonus";
const size_t NORMAL_GAME_ARGUMENT_COUNT = 1;
const size_t BONUS_GAME_ARGUMENT_COUNT = 2;
const size_t EXIT_WRONG_ARGUMENTS = 2;

int main(int argc, char* argv[])
{
  if (argc == NORMAL_GAME_ARGUMENT_COUNT)
  {
    Game& game = Game::instance();
    return game.run();
  }
  else if (argc == BONUS_GAME_ARGUMENT_COUNT)
  {
    if (std::string(argv[1]) == BONUS_ARGUEMT)
    {
      Game& game = Game::instance();
      game.setBonus(true);
      return game.run();
    }
  }
  std::cout << WRONG_ARGUMENTS << std::endl;
  return EXIT_WRONG_ARGUMENTS;
}
