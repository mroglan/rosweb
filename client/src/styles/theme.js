import { createTheme } from "@mui/material";

export const theme = createTheme({
    palette: {
        primary: {
            main: 'hsl(246,79%,44%)',
            dark: 'hsl(246,79%,30%)'
        },
        background: {
            default: 'hsl(246,100%,98%)',
            paper: '#fff'
        }
    }
})