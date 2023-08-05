import { Box, Divider, Grid, IconButton, Paper, Typography } from "@mui/material";
import { GROUP_COLORS } from "./Main";
import UnfoldMoreIcon from '@mui/icons-material/UnfoldMore';
import UnfoldLessIcon from '@mui/icons-material/UnfoldLess';

export default function WaypointCard({controls, setControls, group,
    num, expanded, unExpand, expand}) {

    const point = controls.waypoints[group][num]

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
            {expanded ? <Box>
                <Box>
                    <Divider />
                </Box>
                <Box ml="50px" mt={2} mr={2}>
                    <Box display="grid" sx={{
                        gridTemplateColumns: '1fr 1fr',
                        gridAutoFlow: 'row',
                        alignItems: 'center'
                    }}>
                        <Box>
                            <Typography variant="h6">
                                Latitude
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.navsat.latitude}
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                Longitude
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.navsat.longitude}
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                Altitude
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.navsat.altitude}
                            </Typography>
                        </Box>
                    </Box>
                </Box>
                <Box mt={2}>
                    <Divider />
                </Box>
                <Box ml="50px" mt={2} mr={2}>
                    <Box display="grid" sx={{
                        gridTemplateColumns: '1fr 1fr',
                        gridAutoFlow: 'row',
                        alignItems: 'center'
                    }}>
                        <Box>
                            <Typography variant="h6">
                                x
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.orientation.x}
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                y
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.orientation.y}
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                z
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.orientation.z}
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                w
                            </Typography>
                        </Box>
                        <Box>
                            <Typography variant="h6">
                                {point.orientation.w}
                            </Typography>
                        </Box>
                    </Box>
                </Box>
                <Box mt={2}>
                    <Divider />
                </Box>
                <Box mt={2} pb={2} ml="50px">
                    <Typography variant="h6" color="error" sx={{cursor: 'pointer'}}>
                        Remove
                    </Typography>
                </Box>
            </Box> : null}
        </Paper>
    )
}