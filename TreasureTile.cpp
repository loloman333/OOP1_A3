//----------------------------------------------------------------------------------------------------------------------
// TreasureTile.cpp
//
// The TreasureTile class is a subclass of "Tile". It represents a treasure on
// the board.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "TreasureTile.hpp"
#include "Treasure.hpp"

#include <algorithm>

TreasureTile::TreasureTile(Treasure* treasure) :
    Tile(TileType::T, calculateRotation(treasure->getTreasureId())), treasure_{treasure}
{
}

TreasureTile::TreasureTile(TileType type, Treasure* treasure) : Tile(type), treasure_{treasure} {}

std::string TreasureTile::createTreasureId()
{
  size_t treasure_id = treasure_->getTreasureId();
  std::string treasure_string = TREASURE_STRING_T;

  if (isSingleDigit(treasure_id))
  {
    treasure_string.append(TREASURE_STRING_0);
  }

  treasure_string.append(std::to_string(treasure_id));
  return treasure_string;
}

Rotation TreasureTile::calculateRotation(size_t treasure_id)
{
  if (std::count(TREASURE_DEG_0.begin(), TREASURE_DEG_0.end(), treasure_id))
  {
    return Rotation::DEG0;
  }
  else if (std::count(TREASURE_DEG_90.begin(), TREASURE_DEG_90.end(), treasure_id))
  {
    return Rotation::DEG90;
  }
  else if (std::count(TREASURE_DEG_180.begin(), TREASURE_DEG_180.end(), treasure_id))
  {
    return Rotation::DEG180;
  }
  else if (std::count(TREASURE_DEG_270.begin(), TREASURE_DEG_270.end(), treasure_id))
  {
    return Rotation::DEG270;
  }
  return Rotation::DEG0;
}

void TreasureTile::foundTreasure()
{
  treasure_->setFound(true);
}

std::vector<std::string> TreasureTile::getTileString()
{
  std::vector<std::string> tile_string = getRawTileString();
  if (getFound())
  {
    return tile_string;
  }

  addTreasureIdToTile(tile_string);
  return tile_string;
}

void TreasureTile::addTreasureIdToTile(std::vector<std::string>& tile_string)
{
  std::string treasure_id = createTreasureId();
  if (tile_string[TREASURE_TILE_ROW][0] == ' ') // check if there is a wall
  {
    tile_string[TREASURE_TILE_ROW].replace(INDEX_WITHOUT_WALL, treasure_id.length(), treasure_id);
  }
  else
  {
    tile_string[TREASURE_TILE_ROW].replace(INDEX_WITH_WALL, treasure_id.length(), treasure_id);
  }
}

bool TreasureTile::hasTreasure()
{
  if (getFound())
  {
    return false;
  }
  return true;
}

bool TreasureTile::isSingleDigit(size_t number)
{
  return (number <= MAX_SINGLE_DIGIT);
}

Treasure* TreasureTile::getTreasure()
{
  return treasure_;
}

bool TreasureTile::getFound()
{
  return treasure_->getFound();
}

void TreasureTile::setTreasure(Treasure* treasure)
{
  treasure_ = treasure;
}

void TreasureTile::setFound(bool found)
{
  treasure_->setFound(found);
}
