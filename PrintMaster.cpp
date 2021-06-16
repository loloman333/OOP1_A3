//----------------------------------------------------------------------------------------------------------------------
// PrintMaster.cpp
//
// The PrintMaster Class manages all outputs that need to be made for a Game
// That means it comes with a bunch of string constants and functions to print them
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "PrintMaster.hpp"
#include "CommandMaster.hpp"
#include "Game.hpp"
#include "Item.hpp"
#include "Player.hpp"
#include "Tile.hpp"
#include "Treasure.hpp"
#include <iostream>

PrintMaster::PrintMaster(Game& game) : game_{game} {};

void PrintMaster::printGame()
{
  std::vector<std::string> ui_lines = generateUILines();

  std::cout << ui_lines[0] << std::endl;
  std::cout << ui_lines[1] << std::endl;
  std::cout << UI_COLUMN_LABELS << std::endl;
  printBoard();
  std::cout << ui_lines[2] << std::endl;
  std::cout << ui_lines[3] << std::endl;

  if (game_.getCommandMaster()->getShowTreasure())
  {
    printTreasure();
  }
}

std::vector<std::string> PrintMaster::generateUILines()
{
  size_t player_index_titles = 0;
  size_t player_index_treasures = 0;

  size_t title_row_at = 0;
  size_t title_row_mod = UI_TITLE_ROW_MOD;

  size_t arrow_base_at = 0;
  size_t arrow_base_mod = UI_ARROW_BASE_MOD;

  std::vector<std::string> lines;

  for (size_t index = 0; index < 4; index++)
  {
    std::string line = "                                                                       ";
    if (index % title_row_mod == title_row_at)
    {
      addPlayerTitlesToLine(line, player_index_titles);
    }
    else
    {
      addTreasureCountersToLine(line, player_index_treasures);
    }

    if (index % arrow_base_mod == arrow_base_at)
    {
      addArrowBasesToLine(line);
    }
    else
    {
      if (index == UI_BOTTOM_ARROW_TIPS_AT)
      {
        addArrowTipsToLine(line, Direction::BOTTOM);
      }
      else if (index == UI_TOP_ARROW_TIPS_AT)
      {
        addArrowTipsToLine(line, Direction::TOP);
      }
    }
    line.erase(line.find_last_not_of(" ") + 1);
    lines.push_back(line);
  }

  return lines;
}

void PrintMaster::addPlayerTitlesToLine(std::string& line, size_t& player_index)
{
  for (size_t times = 0; times < 2; times++)
  {
    if (player_index < game_.getPlayers().size())
    {
      std::string title = UI_PLAYER_BASE;
      title += game_.getPlayers()[player_index]->getPlayerColorAsString();
      title += "(";
      title += static_cast<char>(game_.getPlayers()[player_index]->getPlayerColor());
      title += ")";

      size_t actual_offset = UI_PLAYER_TITLE_OFFSET;
      if (player_index == 3)
      {
        actual_offset += UI_UP_ARROWS_EXTRA_SPACE;
      }

      line.replace(actual_offset * times, title.length(), title);
      player_index++;
    }
  }
}

void PrintMaster::addTreasureCountersToLine(std::string& line, size_t& player_index)
{
  for (size_t times = 0; times < 2; times++)
  {
    if (player_index < game_.getPlayers().size())
    {
      std::string treasure_counter = UI_TREASURE_COUNTER;
      treasure_counter.append(std::to_string(game_.getPlayers()[player_index]->getNrFoundTreasures()));
      treasure_counter.append("/");
      treasure_counter.append(std::to_string(game_.getTreasures().size() / game_.getPlayers().size()));

      line.replace(UI_PLAYER_TITLE_OFFSET * times, treasure_counter.length(), treasure_counter);
      player_index++;
    }
  }
}

void PrintMaster::addArrowBasesToLine(std::string& line)
{
  line.replace(UI_ARROW_OFFSET, UI_ARROW_BASES.length(), UI_ARROW_BASES);
}

void PrintMaster::addArrowTipsToLine(std::string& line, Direction direction)
{
  std::string arrows;
  if (direction == Direction::BOTTOM)
  {
    arrows = UI_DOWN_ARROWS;
  }
  else if (direction == Direction::TOP)
  {
    arrows = UI_UP_ARROWS;
  }
  line.replace(UI_ARROW_OFFSET, arrows.length(), arrows);
}

void PrintMaster::printBoard()
{
  size_t row_label_index = UI_FIRST_ROW_LABEL_INDEX;
  size_t line_index = 0;
  size_t row_index = 1;
  for (std::vector<Tile*> row : game_.getBoard())
  {
    std::vector<std::vector<std::string>> tile_strings;
    for (Tile* tile : row)
    {
      tile_strings.push_back(tile->getTileString());
    }

    for (size_t tile_line_index = 0; tile_line_index < TILE_HEIGHT; tile_line_index++)
    {
      bool print_arrow = false;

      printLeftUI(row_index, line_index, row_label_index, print_arrow);
      printTilesOfLine(tile_strings, tile_line_index);
      printRightUI(print_arrow);
      line_index++;
    }
  }
}

void PrintMaster::printLeftUI(size_t& row_index, size_t line_index, size_t& row_label_index, bool& print_arrow)
{
  if (line_index == row_label_index)
  {
    row_label_index += 5;
    if (row_index % 2 == 0)
    {
      print_arrow = true;
      std::cout << UI_ARROW_RIGHT;
    }
    else
    {
      std::cout << "   ";
    }
    std::cout << row_index++;
  }
  else
  {
    std::cout << "    ";
  }
}

void PrintMaster::printTilesOfLine(std::vector<std::vector<std::string>>& tile_strings, size_t tile_line_index)
{
  for (std::vector<std::string> tile_rows : tile_strings)
  {
    std::cout << tile_rows[tile_line_index];
  }
}

void PrintMaster::printRightUI(bool print_arrow)
{
  if (print_arrow)
  {
    std::cout << UI_ARROW_LEFT;
  }
  std::cout << std::endl;
}

void PrintMaster::printWin(std::string color)
{
  std::cout << UI_WIN_1 << color << UI_WIN_2 << std::endl;
}

void PrintMaster::resetUI()
{
  std::cout << UI_CLEAR;
  printGame();
}

void PrintMaster::printGameIfNecessary()
{
  if (game_.getCommandMaster()->getShowGamefield())
  {
    game_.getPrintMaster()->printGame();
  }
}

void PrintMaster::printFreeTile()
{
  std::cout << FREE_TILE << std::endl;
  game_.getFreeTile()->print();
}

void PrintMaster::printTreasure()
{
  std::vector<Treasure*> covered_stack = game_.getCurrentPlayer()->getCoveredStackRef();
  if (covered_stack.empty())
  {
    std::cout << ALL_TREASURES_FOUND << std::endl;
  }
  else
  {
    Treasure* next_treasure = covered_stack.back();
    std::string treasure_name = next_treasure->getName();
    size_t treasure_id = next_treasure->getTreasureId();
    std::string id_string = "";

    if (treasure_id < 10)
    {
      id_string = "0";
    }
    id_string += std::to_string(treasure_id);

    std::cout << CURRENT_TREASURE << treasure_name << TREASURE_NUMBER << id_string << std::endl;
  }
}

void PrintMaster::tiedUpPlayer(std::string player)
{
  std::cout << ROPE_TIED_1 << player << ROPE_TIED_2 << std::endl;
}

void PrintMaster::playerRoped()
{
  std::cout << ROPE_CANT_MOVE << std::endl;
}

void PrintMaster::ladderUsed()
{
  std::cout << UI_LADDER_PROMPT << std::endl;
}

void PrintMaster::noItem()
{
  std::cout << UI_NO_ITEM << std::endl;
}

void PrintMaster::printBomb()
{
  std::cout << UI_BOMB << std::endl;
}

void PrintMaster::noPlayerToTieUp()
{
  std::cout << ROPE_NO_PLAYER << std::endl;
}

void PrintMaster::whereWall()
{
  std::cout << UI_WHERE_WALL;
}

void PrintMaster::wallExists()
{
  std::cout << UI_WALL_EXISTS << std::endl;
}

void PrintMaster::wallBuilt()
{
  std::cout << UI_WALL_BUILT << std::endl;
}

void PrintMaster::invalidCommand(std::string command)
{
  std::cout << COMMAND_INVALID << "\"" << command << "\"" << std::endl;
}

void PrintMaster::invalidParameter(std::string parameter)
{
  std::cout << COMMAND_INVALID_PARAMETER << "\"" << parameter << "\"" << std::endl;
}

void PrintMaster::wrongNumberArguments()
{
  std::cout << COMMAND_WRONG_NUMBER_ARGUMENTS << std::endl;
}

void PrintMaster::commandTakesNoArguments()
{
  std::cout << COMMAND_TAKES_NO_ARGUMENTS << std::endl;
}

void PrintMaster::commandNotAllowed(std::vector<std::string> tokens)
{
  std::string invalid_command = "";
  for (size_t index = 0; index < tokens.size(); index++)
  {
    invalid_command += (tokens[index] + " ");
  }
  commandNotAllowed(invalid_command.substr(0, invalid_command.length() - 1));
}

void PrintMaster::commandNotAllowed(std::string command)
{
  std::cout << "\"" << command << "\"" << COMMAND_NOT_ALLOWED << std::endl;
}

void PrintMaster::impossibleMove()
{
  std::cout << IMPOSSIBLE_MOVE << std::endl;
}

void PrintMaster::invalidInsertPosition()
{
  std::cout << INVALID_POSITION << std::endl;
}

void PrintMaster::printAIName()
{
  std::cout << UI_INFO << UI_AI_NAME << std::endl;
}
