import { Box, Grid, IconButton, Paper, Typography } from "@mui/material";
import { GROUP_COLORS } from "./Main";
import UnfoldMoreIcon from '@mui/icons-material/UnfoldMore';
import UnfoldLessIcon from '@mui/icons-material/UnfoldLess';

export default function WaypointCard({controls, setControls, group,
    num, expanded, unExpand, expand}) {

    return (
        <Paper elevation={3}>
            <Box>
                <Grid container alignItems="center">
                    <Grid item>
                        <Box height={100} width={50} bgcolor={GROUP_COLORS[controls.groupColors[group]]}
                        display="flex" sx={{alignItems: 'center', justifyContent: 'center'}}>
                            <Typography variant="h6" color="#fff">
                                {group}
                            </Typography>
                        </Box>
                    </Grid>
                    <Grid item flex={1}>
                        <Box ml={2}>
                            <Typography variant="h6">
                                Point {num + 1}
                            </Typography>
                        </Box>
                    </Grid>
                    <Grid item>
                        {!expanded ? <IconButton onClick={() => expand(group, num)}>
                            <UnfoldMoreIcon />
                        </IconButton> : <IconButton onClick={() => unExpand()}>
                            <UnfoldLessIcon />     
                        </IconButton>}
                    </Grid>
                </Grid>
            </Box>
        </Paper>
    )
}