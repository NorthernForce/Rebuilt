import { useEntry } from "@frc-web-components/react";
import { CheckCircleOutline, ErrorOutline } from "@mui/icons-material";
import { Avatar, Box, Chip } from "@mui/material";
import { createTheme } from "@mui/material/styles";
import { useContext } from "react";
import { NavigatableContext } from "./NavigatableTreeDisplay";

const theme = createTheme({
  palette: {
    primary: {
      main: '#1976d2', // Blue
    },
    success: {
      main: '#4caf50', // Green for connected
    },
    error: {
      main: '#f44336', // Red for disconnected
    },
  },
  typography: {
    fontFamily: 'Inter, sans-serif', // Use Inter font
  },
  components: {
    MuiChip: {
      styleOverrides: {
        root: {
          borderRadius: '9999px', // Fully rounded corners
          padding: '0.5rem 1rem',
          fontSize: '1rem',
          fontWeight: 'bold',
          transition: 'transform 0.2s ease-in-out',
          '&:hover': {
            transform: 'scale(1.05)',
          },
        },
        label: {
          paddingLeft: '0.5rem',
          paddingRight: '0.5rem',
        },
        avatar: {
          width: '28px',
          height: '28px',
          fontSize: '1rem',
          backgroundColor: 'transparent', // Make avatar background transparent
        },
      },
    },
    MuiLink: {
      styleOverrides: {
        root: {
          textDecoration: 'none', // Remove underline from link
          '&:hover': {
            textDecoration: 'none', // Keep no underline on hover
          },
        },
      },
    },
  },
});

interface MotorChipProps {
    motorId: number
}

function MotorChip({ motorId }: MotorChipProps) {
    const context = useContext(NavigatableContext);
    const [isConnected] = useEntry(`FWC/Motors/${motorId}/Connected`, false);
    const [motorType] = useEntry(`FWC/Motors/${motorId}/Type`, 'Unknown');
    const path = `motors/${motorId}`;
    const color = isConnected ? 'success' : 'error';
    const statusIcon = isConnected ? <CheckCircleOutline /> : <ErrorOutline />;
    const statusText = isConnected ? 'Connected' : 'Disconnected';
    return (
        <Box className="flex justify-center items-center p-4">
            <Chip
                avatar={<Avatar sx={{ color: theme.palette[color].main }}>{statusIcon}</Avatar>}
                label={`Motor ${motorId} - ${motorType} (${statusText})`}
                color={color}
                variant="filled"
                clickable
                className="shadow-lg hover:shadow-xl transition-shadow duration-300"
                onClick={() => {
                    context.setSelectedPath(path);
                }}
            />
        </Box>
    )
}

export default MotorChip;