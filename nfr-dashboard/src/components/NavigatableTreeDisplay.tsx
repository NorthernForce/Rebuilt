import { Box } from "@mui/material";
import { SimpleTreeView, TreeItem } from "@mui/x-tree-view";
import { createContext, useState } from "react";

interface NaivgatableTreeItem {
    id: string; // Unique identifier for the tree item
    label: string; // Display label for the tree item
    children?: NaivgatableTreeItem[]; // Optional children for nested items
    display?: React.ReactNode; // Optional custom display component for the item
}

interface NavigatableTreeDisplayProps {
    items: NaivgatableTreeItem[];
}

function getTreeItem(item: NaivgatableTreeItem, prefix: string | undefined = undefined): React.ReactNode {
    const path = prefix ? `${prefix}/${item.id}` : item.id;
    return (
        <TreeItem itemId={path} label={item.label}>
            {item.children?.map(child => getTreeItem(child, path))}
        </TreeItem>
    );
}

function getTreeItemByPath(items: NaivgatableTreeItem[], path: string): NaivgatableTreeItem | null {
    const firstElement = path.split('/')[0];
    for (const item of items) {
        if (item.id === firstElement) {
            if (path === item.id) {
                return item; // Found the exact match
            }
            if (item.children) {
                const childResult = getTreeItemByPath(item.children, path.slice(firstElement.length + 1));
                if (childResult) {
                    return childResult; // Found in children
                }
            }
        }
    }
    return null;
}

export const NavigatableContext = createContext<{
    selectedPath: string | null;
    setSelectedPath: (path: string | null) => void;
}>({
    selectedPath: null,
    setSelectedPath: () => {},
});

function NavigatableTreeDisplay({ items }: NavigatableTreeDisplayProps) {
    const [selectedPath, setSelectedPath] = useState<string | null>(null);
    return (
        <Box sx={{display: 'flex', width: '100%', height: '100%'}}>
            <Box sx={(theme) => ({
                width: 250,
                bgcolor: theme.palette.mode === 'dark' ? 'grey.800' : 'grey.200',
                padding: 1,
                borderRight: '1px solid',
                borderColor: theme.palette.mode === 'dark' ? 'grey.700' : 'grey.300',
                height: '100%', // Ensures the box expands the full height
                overflowY: 'auto', // Adds scrolling if content overflows
            })}>
                <SimpleTreeView onSelectedItemsChange={(_, nodeId) => {
                    console.log(`Selected node: ${nodeId}`);
                    setSelectedPath(nodeId);
                }}>
                    {items.map(item => getTreeItem(item))}
                </SimpleTreeView>
            </Box>
            <Box sx={{flexGrow: 1, padding: 2}}>
                <NavigatableContext.Provider value={{selectedPath, setSelectedPath}}>
                    {(selectedPath && getTreeItemByPath(items, selectedPath)?.display) || 
                        <Box sx={{padding: 2, textAlign: 'center'}}>
                            <h3>Invalid path: {selectedPath}</h3>
                            <p>Please select a valid item from the tree.</p>
                        </Box>
                    }
                </NavigatableContext.Provider>
            </Box>
        </Box>
    )
}

export default NavigatableTreeDisplay;