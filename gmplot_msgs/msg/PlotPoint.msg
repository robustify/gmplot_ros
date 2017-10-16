float64 lat                     # Latitude of the graphic
float64 lon                     # Longitude of the graphic

string text                     # Text label of the marker. Only used for
                                # TEXT_LABEL and MARKER_WITH_TEXT types

uint8 size                      # Size of the graphic placed on map. Only used for
                                # SCATTER_POINT and LINE types
                                
string color                    # Color of graphic placed on map
                                #   Single character (b, g, r, c, m, y, k, w)
                                #   HTML color names (aliceblue, chartreuse, etc.)
                                #   HTML color codes (#123ABC, #FFFFFF, etc.)

uint8 SCATTER_POINT=0           # Individual circles with radius equal to the size field
uint8 LINE=1                    # Connected dots with line width equal to the size field
uint8 MARKER=2                  # Pin marker
uint8 TEXT_LABEL=3              # Text marker
uint8 MARKER_WITH_TEXT=4        # Both pin and text
uint8 type
