//----------------------------------------------------------------------------------------------------------------------
// ItemTile.cpp
//
// The Item Tile class represents a tile that contains an item that can be picked up.
// It is a subclass of "Tile" and implements all necessary functions.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "ItemTile.hpp"
#include "Item.hpp"

ItemTile::ItemTile(TileType type, Item* item) : Tile(type, Rotation::DEG0), item_{item} {}

Item* ItemTile::getItem()
{
  return item_;
}

bool ItemTile::getFound()
{
  return item_->getFound();
}

void ItemTile::setFound(bool found)
{
  item_->setFound(found);
}

bool ItemTile::hasItem()
{
  if (getFound())
  {
    return false;
  }
  return true;
}

std::vector<std::string> ItemTile::getTileString()
{
  std::vector<std::string> tileString = getRawTileString();
  if (getFound())
  {
    return tileString;
  }

  std::string item_name = item_->getItemTypeAsString();
  if (tileString[ITEM_TILE_ROW][0] == ' ')
  {
    tileString[ITEM_TILE_ROW].replace(INDEX_WITHOUT_WALL, item_name.length(), item_name);
  }
  else
  {
    tileString[ITEM_TILE_ROW].replace(INDEX_WITH_WALL, item_name.length(), item_name);
  }

  return tileString;
}