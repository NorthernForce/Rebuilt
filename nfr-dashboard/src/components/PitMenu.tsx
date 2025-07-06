import { Expand, ExpandLess, ExpandMore } from "@mui/icons-material";
import { Box, Collapse, List, ListItem, ListItemButton } from "@mui/material";
import React, { useState } from "react";

interface PitMenuItem {
    key: string;
    label: React.ReactNode;
    value: React.ReactNode;
    children?: PitMenuItem[];
}

interface PitMenuProps {
    menuToDisplay: PitMenuItem[];
    selectedMenu: string;
    setSelectedMenu: (menu: string) => void;
}

function renderMenuItem({key, label, children}: PitMenuItem, selectedMenu: string, setSelectedMenu: (menu: string) => void, keyPrefix?: string): React.ReactNode {
    const doesHaveChildren = children && Array.isArray(children) && children.length > 0;
    const [hasExpanded, setHasExpanded] = useState(false);
    return (
        <>
            <ListItem disablePadding>
                <ListItemButton
                    selected={selectedMenu === (keyPrefix ? `${keyPrefix}/${key}` : key)}
                    onClick={() => {
                        setSelectedMenu((keyPrefix ? `${keyPrefix}/${key}` : key));
                        setHasExpanded(!hasExpanded);
                    }}
                    sx={{ mx: 2, my: 0.5 }}
                >
                    {label}
                    {doesHaveChildren && (hasExpanded ? <ExpandLess /> : <ExpandMore />)}
                </ListItemButton>
            </ListItem>
            {doesHaveChildren && (
                <Collapse in={hasExpanded} timeout="auto" unmountOnExit>
                    <List component="div" disablePadding>
                        {Array.isArray(children) &&
                            children.map((child) => renderMenuItem(child, selectedMenu, setSelectedMenu, keyPrefix ? `${keyPrefix}/${key}` : key))}
                    </List>
                </Collapse>
            )}
        </>
    );
}

function PitMenu({ menuToDisplay, selectedMenu, setSelectedMenu }: PitMenuProps) {
    return (
        <>
            <Box sx={{ width: 280, bgcolor: 'background.paper', height: '100%' }}>
                <List component="nav">
                    {menuToDisplay.map((menu) => (
                        renderMenuItem(menu, selectedMenu, setSelectedMenu)
                    ))}
                </List>
            </Box>
            <Box sx={{ flexGrow: 1 }} />
            <Box sx={{ p: 2 }}>
                {menuToDisplay.find(menu => menu.key === selectedMenu)?.value}
            </Box>
        </>
    );
}

export default PitMenu;