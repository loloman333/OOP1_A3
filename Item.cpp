//----------------------------------------------------------------------------------------------------------------------
// Item.cpp
//
// The Item class represent an item that can be collected and used by players
// Implements the functionality for using items
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "Item.hpp"
#include "CommandMaster.hpp"
#include "Game.hpp"
#include "ItemTile.hpp"
#include "Player.hpp"
#include "PrintMaster.hpp"
#include "Tile.hpp"
#include <algorithm>

Item::Item(ItemType item_type) : item_type_{item_type} {};

ItemType Item::getItemType()
{
  return item_type_;
}

std::string Item::getItemTypeAsString()
{
  switch (item_type_)
  {
    case ItemType::BRICKS:
      return "BRI";

    case ItemType::DYNAMITE:
      return "DYN";

    case ItemType::ROPE:
      return "ROP";

    case ItemType::LADDER:
      return "LAD";
  }
}

bool Item::getFound()
{
  return found_;
}

void Item::setFound(bool found)
{
  found_ = found;
}

void Item::use()
{
  Game& game = Game::instance();
  switch (item_type_)
  {
    case ItemType::BRICKS:
      useBricks(game);
      break;
    case ItemType::DYNAMITE:
      useDynamite(game);
      break;
    case ItemType::LADDER:
      useLadder(game);
      break;
    case ItemType::ROPE:
      useRope(game);
      break;
  }
}

void Item::useBricks(Game& game)
{
  std::string answer;
  std::vector<std::string> tokens;

  do
  {
    game.getPrintMaster()->whereWall();
    std::getline(std::cin, answer);
    tokens = game.getCommandMaster()->tokenize(answer);
  } while (tokens.size() != 1 || !isValidResponse(tokens[0]));

  Direction direction = getDirectionFromString(tokens[0]);

  Player* current_player = game.getCurrentPlayer();
  Tile* current_tile = game.getBoard()[current_player->getRow()][current_player->getCol()];

  setBricksInDirection(direction, current_tile);
}

bool Item::isValidResponse(std::string response)
{
  return std::find(WALL_VALID_RESPONSES.begin(), WALL_VALID_RESPONSES.end(), response) != WALL_VALID_RESPONSES.end();
}

Direction Item::getDirectionFromString(std::string string)
{
  if (string == "top")
  {
    return Direction::TOP;
  }
  else if (string == "left")
  {
    return Direction::LEFT;
  }
  else if (string == "bottom")
  {
    return Direction::BOTTOM;
  }
  else if (string == "right")
  {
    return Direction::RIGHT;
  }
  else
  {
    return Direction::UNDEFINED;
  }
}

void Item::setBricksInDirection(Direction direction, Tile* tile)
{
  Game& game = Game::instance();

  if (tile->addWallInDirection(direction))
  {
    game.getPrintMaster()->printGame();
    game.getPrintMaster()->wallBuilt();
    game.getCurrentPlayer()->setItem(nullptr);
    setFound(false);
    
  }
  else
  {
    game.getPrintMaster()->wallExists();
  }
}

void Item::useDynamite(Game& game)
{
  int row = game.getCurrentPlayer()->getRow();
  int col = game.getCurrentPlayer()->getCol();
  game.getBoard()[row][col]->setType(TileType::X);

  if (row + 1 < static_cast<int>(BOARD_SIZE))
  {
    game.getBoard()[row + 1][col]->removeWallInDirection(Direction::TOP);
  }
  if ((col - 1) >= 0)
  {
    game.getBoard()[row][col - 1]->removeWallInDirection(Direction::RIGHT);
  }
  if ((row - 1) >= 0)
  {
    game.getBoard()[row - 1][col]->removeWallInDirection(Direction::BOTTOM);
  }
  if (col + 1 < static_cast<int>(BOARD_SIZE))
  {
    game.getBoard()[row][col + 1]->removeWallInDirection(Direction::LEFT);
  }

  game.getPrintMaster()->printGame();
  game.getPrintMaster()->printBomb();
  game.getCurrentPlayer()->setItem(nullptr);
  setFound(false);
}

void Item::useLadder(Game& game)
{
  game.getPrintMaster()->ladderUsed();
  game.getCurrentPlayer()->setUsingLadder(true);
  game.getCurrentPlayer()->setItem(nullptr);
  setFound(false);
}

void Item::useRope(Game& game)
{
  std::vector<std::vector<Tile*>>& board = game.getBoard();
  Player* current_player = game.getCurrentPlayer();
  Tile* current_tile = board[current_player->getRow()][current_player->getCol()];

  if (current_tile->getPlayers().size() > 1)
  {
    current_player->getItem()->setFound(false);
    current_player->setItem(nullptr);

    for (PlayerColor color : Player::player_colors_)
    {
      if (current_tile->isPlayerColorOnTile(color) && color != current_player->getPlayerColor())
      {
        Player* player = current_tile->getPlayer(color);
        player->setTied(true);
        std::string player_color = player->getPlayerColorAsString();
        std::transform(player_color.begin(), player_color.end(), player_color.begin(), ::toupper);
        game.getPrintMaster()->tiedUpPlayer(player_color);
      }
    }
  }
  else
  {
    game.getPrintMaster()->noPlayerToTieUp();
  }
}
