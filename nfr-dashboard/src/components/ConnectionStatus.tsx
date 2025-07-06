import { useEntry } from "@frc-web-components/react";
import { Box, Chip, Typography, styled, alpha } from '@mui/material';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import CancelIcon from '@mui/icons-material/Cancel';

// Styled Box for the container to apply conditional background and border
const ConnectionContainer = styled(Box)(({ theme, isConnected }: { theme?: any; isConnected: boolean }) => ({
    display: 'flex',
    alignItems: 'center',
    gap: theme.spacing(1), // Reduced spacing
    padding: theme.spacing(0.5, 1.5), // Reduced padding
    borderRadius: theme.shape.borderRadius,
    boxShadow: theme.shadows[0], // Less pronounced shadow
    transition: 'background-color 0.3s ease-in-out, border-color 0.3s ease-in-out',
    
    backgroundColor: theme.palette.background.paper,
    border: `1px solid ${theme.palette.divider}`,

    ...(isConnected && {
        backgroundColor: alpha(theme.palette.success.main, 0.1),
        border: `1px solid ${theme.palette.success.dark}`,
    }),
    ...(!isConnected && {
        backgroundColor: alpha(theme.palette.error.main, 0.1),
        border: `1px solid ${theme.palette.error.dark}`,
    }),
}));

interface ConnectionStatusProps {
    sourceKey: string;
    changeConnectionStatus?: (status: boolean) => void;
}

function ConnectionStatus({ sourceKey, changeConnectionStatus }: ConnectionStatusProps) {
    const [connected] = useEntry(`${sourceKey}/connected`, false);
    const [heartbeat] = useEntry(`${sourceKey}/heartbeat`, 0);
    const [robotName] = useEntry(`${sourceKey}/robotName`, 'Unknown Robot');

    const isConnected = connected && heartbeat > Date.now() - 5000; 
    const ping = isConnected ? Math.round((Date.now() - heartbeat) / 1000) : null;

    changeConnectionStatus?.(isConnected);

    return (
        <ConnectionContainer isConnected={isConnected}>
            {/* Status Indicator and Text */}
            <Chip
                icon={isConnected ? <CheckCircleIcon fontSize="small" /> : <CancelIcon fontSize="small" />} // Smaller icon
                label={isConnected ? 'Connected' : 'Disconnected'}
                color={isConnected ? 'success' : 'error'}
                variant="outlined"
                size="small" // Smaller chip size
                sx={{
                    backgroundColor: (theme) => isConnected
                        ? alpha(theme.palette.success.main, 0.2)
                        : alpha(theme.palette.error.main, 0.2),
                    color: isConnected ? 'success.dark' : 'error.dark',
                    borderColor: isConnected ? 'success.dark' : 'error.dark',
                    '& .MuiChip-icon': {
                        color: 'inherit',
                    },
                }}
            />
            
            {/* Robot Name */}
            <Typography 
                variant="body2" // Smaller text variant
                sx={{ 
                    flexGrow: 1,
                    fontWeight: 'bold',
                    color: isConnected ? 'text.primary' : 'text.secondary', 
                    opacity: isConnected ? 1 : 0.8,
                    transition: 'opacity 0.3s ease-in-out',
                    whiteSpace: 'nowrap', // Prevent wrapping
                    overflow: 'hidden', // Hide overflow
                    textOverflow: 'ellipsis', // Show ellipsis for truncated text
                }}
            >
                {robotName}
            </Typography>

            {/* Ping Display */}
            {ping !== null && (
                <Typography 
                    variant="caption" // Smaller text variant for ping
                    sx={{ 
                        color: isConnected ? 'text.secondary' : 'text.disabled',
                        opacity: isConnected ? 1 : 0.7,
                        transition: 'opacity 0.3s ease-in-out'
                    }}
                >
                    {ping}s
                </Typography>
            )}
        </ConnectionContainer>
    );
}

export default ConnectionStatus;