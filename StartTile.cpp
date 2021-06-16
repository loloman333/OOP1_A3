//----------------------------------------------------------------------------------------------------------------------
// StartTile.cpp
//
// The Start Tile class represents a tile in the corner of the board where players can start the game.
// It is a subclass of "Tile" and implements all necessary functions
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "StartTile.hpp"
#include "Player.hpp"

StartTile::StartTile(PlayerColor player_color) : Tile{TileType::L}, player_color_{player_color}
{
  Rotation rotation;
  switch (player_color)
  {
    case PlayerColor::GREEN:
      rotation = Rotation::DEG0;
      break;
    case PlayerColor::BLUE:
      rotation = Rotation::DEG90;
      break;
    case PlayerColor::YELLOW:
      rotation = Rotation::DEG180;
      break;
    case PlayerColor::RED:
      rotation = Rotation::DEG270;
      break;
  }

  setRotation(rotation);
}

PlayerColor StartTile::getPlayerColor()
{
  return player_color_;
}

std::vector<std::string> StartTile::getTileString()
{
  std::vector<std::string> tileString = getRawTileString();
  std::string playerBase{'(', static_cast<char>(getPlayerColor()), ')'};

  if (tileString[PLAYER_TILE_ROW][0] == ' ')
  {
    tileString[PLAYER_TILE_ROW].replace(INDEX_WITHOUT_WALL, playerBase.length(), playerBase);
  }
  else
  {
    tileString[PLAYER_TILE_ROW].replace(INDEX_WITH_WALL, playerBase.length(), playerBase);
  }
  return tileString;
}