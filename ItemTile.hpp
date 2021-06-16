//----------------------------------------------------------------------------------------------------------------------
// ItemTile.hpp
//
// The Item Tile class represents a tile that contains an item that can be picked up.
// It is a subclass of "Tile" and implements all necessary functions.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#ifndef A2_ITEM_TILE_HPP
#define A2_ITEM_TILE_HPP

#include "Tile.hpp"
#include <string>
#include <vector>

const size_t ITEM_TILE_ROW = 2;

class Item;

class ItemTile : public Tile
{
private:
  Item* item_;

public:

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Constructor for the ItemTile class
  /// Takes a TileType and item pointer
  ///
  /// @param type The TileType the Tile should have
  /// @param item Pointer to the item the tile should hold
  ///
  ItemTile(TileType type, Item* item);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// destructor of the ItemTile class
  //
  ~ItemTile() = default;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// copy constructor of the ItemTile class
  ///
  /// @param item_tile the item_tile to be copied
  //
  ItemTile(const ItemTile& item_tile) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// assignment operator of the ItemTile class
  ///
  /// @param item_tile the item_tile to be copied
  //
  ItemTile& operator=(const ItemTile& item_tile) = delete;

  //--------------------------------------------------------------------------------------------------------------------
  /// Getter for item_
  ///
  /// @return A pointer to the item that this tile holds
  //
  Item* getItem();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Getter for found_ of the item_
  ///
  /// @return A boolean if the item that this tile holds is currently found
  //
  bool getFound();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Setter for found_ of the item_
  ///
  /// @param collected A boolean if the item that this tile holds should be found or not
  //
  void setFound(bool collected);

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Returns if the tile has an item that can be collected currently
  ///
  /// @return A boolean if the tile has an item that can be collected currently
  //
  bool hasItem();

  //--------------------------------------------------------------------------------------------------------------------
  ///
  /// Implemented getTileString function of "Tile" superclass
  /// Returns a vector of string for each line of the tile to print it.
  ///
  /// @return All the lines of the tile as vector of strings
  //
  std::vector<std::string> getTileString();
};

#endif // A2_ITEM_TILE_HPP
