function scenario = make_scenario2()
arrivalTimes = load('arrivalTimes.mat').arrivalTimes;
velocities = load('velocities.mat').velocities;
directions = load('directions.mat').directions;
exitTimes = load('exitTimes.mat').exitTimes;
stopTime = max(exitTimes);

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [325 175 0;
    325 285 0];
roadWidth = 10;
road(scenario, roadCenters, roadWidth, 'Name', 'Road');
roadCenters = [270 230 0;
    380 230 0];
roadWidth = 10;
road(scenario, roadCenters, roadWidth, 'Name', 'Road1');

for i=1:length(arrivalTimes)
    direction = string(directions(i));
    travel_trajectory = position_trajectory([325 230], direction, 50, 10, 1);
    velocity_trajectory = [velocities(i,:) repmat(velocities(i,end),1,length(travel_trajectory)-length(velocities(i,:)))];
    spawn_time = arrivalTimes(i);
    if direction == "NS"
        spawn_position = [380 230 0];
        yaw=0;
    else
        spawn_position = [325 175 0];
        yaw=-90;
    end
    car = vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 2, ...
    'Width', 1.8, ...
    'Height', 1.4, ...
    'Yaw', yaw, ...
    'EntryTime', spawn_time, ...    
    'ExitTime', exitTimes(i), ...
    'Position', spawn_position, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', strcat('Car',string(i)));
trajectory(car, travel_trajectory, velocity_trajectory);
end

scenario.SampleTime = 0.2;
scenario.StopTime = stopTime;

plot(scenario)
while advance(scenario)
pause(0.1)
end


