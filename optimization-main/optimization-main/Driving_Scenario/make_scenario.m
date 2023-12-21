function scenario = make_scenario()
arrivalTimes = load('arrivalTimes.mat').arrivalTimes;
velocities = load('velocities.mat').velocities;
directions = load('directions.mat').directions;
exitTimes = load('exitTimes.mat').exitTimes;
stopTime = max(exitTimes);

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [0 500 0;
    650 500 0];
laneSpecification = lanespec(1, 'Width', 49.85);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

roadCenters = [325 175 0;
    325 825 0];
roadWidth = 50;
road(scenario, roadCenters, roadWidth, 'Name', 'Road1');

for i=1:length(arrivalTimes)
    direction = string(directions(i));
    travel_trajectory = position_trajectory([325 500], direction, 300, 50, 5);
    velocity_trajectory = [velocities(i,:) repmat(velocities(i,end),1,length(travel_trajectory)-length(velocities(i,:)))];
    spawn_time = arrivalTimes(i);
    if direction == "NS"
        spawn_position = [650 500 0];
        yaw=0;
    else
        spawn_position = [325 175 0];
        yaw=-90;
    end
    car = vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 20, ...
    'Width', 18, ...
    'Height', 14, ...
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
f = figure;
plot(scenario)
while advance(scenario)
    figure(f)
    bar([scenario.Actors(1).Velocity(2) scenario.Actors(2).Velocity(2) -scenario.Actors(3).Velocity(1) -scenario.Actors(4).Velocity(1)])
    ylim([0 25])
    pause(0.1)
end


