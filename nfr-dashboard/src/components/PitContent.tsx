import React, { useState } from 'react';
import {
  AppBar,
  Toolbar,
  Typography,
  Box,
  Drawer,
  List,
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Collapse,
  Paper,
  CssBaseline,
  useMediaQuery,
  IconButton,
  createTheme,
  ThemeProvider
} from '@mui/material';
import {
  ExpandLess,
  ExpandMore,
  Menu as MenuIcon,
  Dashboard as DashboardIcon,
  Build as BuildIcon,
  Sensors as SensorsIcon,
  Settings as SettingsIcon,
  DirectionsCar as DrivetrainIcon,
  Input as IntakeIcon,
  SportsScore as ShooterIcon,
  VerticalAlignTop as ClimberIcon,
  Memory as MotorIcon,
  Tune as PIDIcon,
  Info as InfoIcon
} from '@mui/icons-material';

// Define a custom theme for a more FRC-like feel
const theme = createTheme({
  palette: {
    primary: {
      main: '#1a237e', // Deep Indigo
    },
    secondary: {
      main: '#ffab00', // Amber
    },
    background: {
      default: '#f5f5f5', // Light grey background
      paper: '#ffffff', // White paper background
    },
  },
  typography: {
    fontFamily: 'Inter, sans-serif',
    h5: {
      fontWeight: 600,
      color: '#1a237e',
    },
    h6: {
      fontWeight: 500,
      color: '#3f51b5',
    },
    body1: {
      fontSize: '0.95rem',
    },
  },
  components: {
    MuiPaper: {
      styleOverrides: {
        root: {
          borderRadius: '12px', // Rounded corners for all Paper components
          boxShadow: '0 4px 12px rgba(0,0,0,0.08)', // Subtle shadow
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: '8px',
        },
      },
    },
    MuiListItemButton: {
      styleOverrides: {
        root: {
          borderRadius: '8px',
          '&:hover': {
            backgroundColor: 'rgba(26, 35, 126, 0.08)', // Light hover for list items
          },
        },
      },
    },
  },
});

// Interface for a single menu item in the hierarchical structure
interface PitMenuItem {
    key: string;
    label: string; // The text label for the menu item
    icon: React.ReactNode; // The MUI icon component for the menu item
    value: React.ReactNode; // The content to display when this item is selected
    children?: PitMenuItem[]; // Optional array for nested menu items
}

interface PitMenuProps {
    menuToDisplay: PitMenuItem[];
    selectedMenu: string;
    setSelectedMenu: (menu: string) => void;
    // State to keep track of expanded parent items, so they remain open when a child is selected
    openSubsystems: { [key: string]: boolean };
    setOpenSubsystems: React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>;
}

/**
 * Recursively renders a single menu item and its children.
 * @param {PitMenuItem} item - The current menu item to render.
 * @param {string} selectedMenu - The currently selected menu item key.
 * @param {(menu: string) => void} setSelectedMenu - Function to set the selected menu item.
 * @param {string} [keyPrefix] - Prefix for the current item's key, used for nested keys.
 * @param {{ [key: string]: boolean }} openSubsystems - State for expanded subsystems.
 * @param {React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>} setOpenSubsystems - Function to set expanded subsystems state.
 * @returns {React.ReactNode} The rendered menu item.
 */
function renderMenuItem(
    item: PitMenuItem,
    selectedMenu: string,
    setSelectedMenu: (menu: string) => void,
    keyPrefix: string | undefined,
    openSubsystems: { [key: string]: boolean },
    setOpenSubsystems: React.Dispatch<React.SetStateAction<{ [key: string]: boolean }>>
): React.ReactNode {
    const { key, label, icon, children } = item;
    const doesHaveChildren = children && Array.isArray(children) && children.length > 0;
    const currentKey = keyPrefix ? `${keyPrefix}/${key}` : key;
    const isSelected = selectedMenu === currentKey;
    const isExpanded = openSubsystems[currentKey];

    const handleItemClick = () => {
        setSelectedMenu(currentKey);
        if (doesHaveChildren) {
            setOpenSubsystems((prev) => ({ ...prev, [currentKey]: !prev[currentKey] }));
        }
    };

    return (
        <React.Fragment key={currentKey}>
            <ListItem disablePadding>
                <ListItemButton
                    selected={isSelected}
                    onClick={handleItemClick}
                    sx={{ mx: 2, my: 0.5 }}
                >
                    <ListItemIcon sx={{ minWidth: 40 }}> {/* Adjust minWidth for consistent icon alignment */}
                        {icon}
                    </ListItemIcon>
                    <ListItemText primary={label} />
                    {doesHaveChildren && (isExpanded ? <ExpandLess /> : <ExpandMore />)}
                </ListItemButton>
            </ListItem>
            {doesHaveChildren && (
                <Collapse in={isExpanded} timeout="auto" unmountOnExit>
                    <List component="div" disablePadding sx={{ pl: 4 }}> {/* Indent children */}
                        {Array.isArray(children) &&
                            children.map((child) =>
                                renderMenuItem(child, selectedMenu, setSelectedMenu, currentKey, openSubsystems, setOpenSubsystems)
                            )}
                    </List>
                </Collapse>
            )}
        </React.Fragment>
    );
}

/**
 * PitMenu component for rendering the hierarchical navigation list.
 * @param {PitMenuProps} props - Props for the PitMenu component.
 * @returns {React.ReactNode} The rendered navigation menu.
 */
function PitMenu({ menuToDisplay, selectedMenu, setSelectedMenu, openSubsystems, setOpenSubsystems }: PitMenuProps) {
    return (
        <Box sx={{ width: 280, bgcolor: 'background.paper', height: '100%' }}>
            <Toolbar sx={{ justifyContent: 'center', py: 2 }}>
                <DashboardIcon sx={{ mr: 1, color: 'primary.main' }} />
                <Typography variant="h6" noWrap component="div" color="primary.main">
                    FRC Pit View
                </Typography>
            </Toolbar>
            <List component="nav">
                {menuToDisplay.map((menu) =>
                    renderMenuItem(menu, selectedMenu, setSelectedMenu, undefined, openSubsystems, setOpenSubsystems)
                )}
            </List>
        </Box>
    );
}

interface PitDisplayProps {
    menuItems: PitMenuItem[];
    selectedMenu: string;
}

/**
 * Helper function to find the value (content) for the selected menu item from the hierarchical menuItems.
 * @param {PitMenuItem[]} items - The array of menu items to search through.
 * @param {string} currentKey - The key of the currently selected menu item.
 * @returns {React.ReactNode | null} The content associated with the selected key, or null if not found.
 */
const findMenuItemValue = (items: PitMenuItem[], currentKey: string): React.ReactNode | null => {
    for (const item of items) {
      const fullKey = item.key;
      // Check if the current item's key matches the selected key
      if (fullKey === currentKey) {
        return item.value;
      }
      // If the item has children, recursively search within them
      if (item.children) {
        const nestedValue = findMenuItemValue(item.children, currentKey);
        if (nestedValue) {
          return nestedValue;
        }
      }
    }
    return null; // Return null if the key is not found
};

/**
 * PitDisplay component for rendering the content associated with the selected menu item.
 * @param {PitDisplayProps} props - Props for the PitDisplay component.
 * @returns {React.ReactNode} The rendered content.
 */
const PitContent = ({ menuItems, selectedMenu }: PitDisplayProps) => {
    const contentToDisplay = findMenuItemValue(menuItems, selectedMenu);

    return (
        <Box
            component="main"
            sx={{
                flexGrow: 1,
                p: 3,
                width: { md: 'calc(100% - 280px)' },
                mt: { xs: '64px', md: '64px' }, // Adjust margin top for app bar height
                pb: 3, // Padding at the bottom
            }}
        >
            {contentToDisplay}
        </Box>
    );
};

export default PitContent;