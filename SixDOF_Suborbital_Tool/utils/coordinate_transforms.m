function varargout = coordinate_transforms(mode, varargin)
%COORDINATE_TRANSFORMS  Reference frame transformations for the 6DOF tool.
%   Wraps Aerospace Toolbox functions with consistent sign conventions.
%
%   USAGE:
%     R = coordinate_transforms('quat2dcm_BI', q)
%       Returns DCM body←inertial (NED) from quaternion [q0,q1,q2,q3]
%
%     R = coordinate_transforms('quat2dcm_IB', q)
%       Returns DCM inertial←body (NED) from quaternion
%
%     [phi,theta,psi] = coordinate_transforms('quat2euler', q)
%       Returns Euler angles (rad) from quaternion (ZYX convention)
%
%     q = coordinate_transforms('euler2quat', phi, theta, psi)
%       Returns quaternion [q0;q1;q2;q3] from Euler angles (rad)
%
%     v_body = coordinate_transforms('ned2body', v_ned, q)
%       Rotates vector from NED frame to body frame
%
%     v_ned = coordinate_transforms('body2ned', v_body, q)
%       Rotates vector from body frame to NED frame
%
%     [lat,lon,h] = coordinate_transforms('ned2llh', xN, xE, xD, lat0, lon0, h0)
%       Converts NED displacement (m) to lat/lon/alt using flat Earth approx.
%
%     [xN,xE,xD] = coordinate_transforms('llh2ned', lat, lon, h, lat0, lon0, h0)
%       Converts lat/lon/alt to NED (flat Earth approx.)
%
%   All angles in radians unless noted.

switch lower(mode)

    %% ====================================================================
    %  QUATERNION → DCM (body ← inertial / NED)
    %  Aerospace Toolbox: quat2dcm([q0,q1,q2,q3]) gives R such that
    %    v_body = R * v_NED
    %  ====================================================================

    case 'quat2dcm_bi'
        q = varargin{1}(:)';
        q = q / norm(q);
        R = quat2dcm(q);   % Aerospace Toolbox (body←NED)
        varargout{1} = R;

    %% ====================================================================
    %  QUATERNION → DCM (inertial ← body)
    %  ====================================================================

    case 'quat2dcm_ib'
        q = varargin{1}(:)';
        q = q / norm(q);
        R = quat2dcm(q)';  % Transpose: NED←body
        varargout{1} = R;

    %% ====================================================================
    %  QUATERNION → EULER ANGLES (ZYX: psi, theta, phi)
    %  ====================================================================

    case 'quat2euler'
        q = varargin{1}(:)';
        q = q / norm(q);
        dcm = quat2dcm(q);
        [psi, theta, phi] = dcm2angle(dcm, 'ZYX');  % Aerospace Toolbox
        varargout{1} = phi;
        varargout{2} = theta;
        varargout{3} = psi;

    %% ====================================================================
    %  EULER ANGLES → QUATERNION (ZYX convention)
    %  ====================================================================

    case 'euler2quat'
        phi   = varargin{1};
        theta = varargin{2};
        psi   = varargin{3};
        q_row = angle2quat(psi, theta, phi, 'ZYX');  % Aerospace Toolbox → [q0,q1,q2,q3]
        varargout{1} = q_row(:);   % Return as column vector

    %% ====================================================================
    %  NED → BODY  (rotate vector from NED to body frame)
    %  ====================================================================

    case 'ned2body'
        v_ned = varargin{1}(:);
        q     = varargin{2}(:)';
        q     = q / norm(q);
        R_BI  = quat2dcm(q);
        varargout{1} = R_BI * v_ned;

    %% ====================================================================
    %  BODY → NED  (rotate vector from body to NED frame)
    %  ====================================================================

    case 'body2ned'
        v_body = varargin{1}(:);
        q      = varargin{2}(:)';
        q      = q / norm(q);
        R_BI   = quat2dcm(q);
        varargout{1} = R_BI' * v_body;

    %% ====================================================================
    %  NED DISPLACEMENT → LAT/LON/ALT  (flat-Earth approximation)
    %  ====================================================================

    case 'ned2llh'
        xN   = varargin{1};  xE  = varargin{2};  xD  = varargin{3};
        lat0 = varargin{4};  lon0= varargin{5};  h0  = varargin{6};

        R_earth = 6378137.0;  % WGS-84 equatorial radius (m)

        lat = lat0 + (xN / R_earth) * (180/pi);
        lon = lon0 + (xE / (R_earth * cos(deg2rad(lat0)))) * (180/pi);
        h   = h0 - xD;   % xD is down → altitude is up

        varargout{1} = lat;
        varargout{2} = lon;
        varargout{3} = h;

    %% ====================================================================
    %  LAT/LON/ALT → NED DISPLACEMENT
    %  ====================================================================

    case 'llh2ned'
        lat  = varargin{1};  lon = varargin{2};  h   = varargin{3};
        lat0 = varargin{4};  lon0= varargin{5};  h0  = varargin{6};

        R_earth = 6378137.0;

        xN = (lat - lat0) * (pi/180) * R_earth;
        xE = (lon - lon0) * (pi/180) * R_earth * cos(deg2rad(lat0));
        xD = -(h - h0);

        varargout{1} = xN;
        varargout{2} = xE;
        varargout{3} = xD;

    %% ====================================================================
    %  WIND FRAME → BODY FRAME (using alpha, beta)
    %  ====================================================================

    case 'wind2body'
        v_wind = varargin{1}(:);
        alpha  = varargin{2};   % rad
        beta   = varargin{3};   % rad

        % Rotation matrix wind→body: R_BW
        ca = cos(alpha); sa = sin(alpha);
        cb = cos(beta);  sb = sin(beta);
        R_BW = [ca*cb, -ca*sb, -sa;
                sb,     cb,    0;
                sa*cb, -sa*sb,  ca];
        varargout{1} = R_BW * v_wind;

    %% ====================================================================
    %  VELOCITY → FLIGHT PATH ANGLE & HEADING
    %  ====================================================================

    case 'vel2fpa'
        u = varargin{1}; v_l = varargin{2}; w = varargin{3};
        V = sqrt(u^2 + v_l^2 + w^2);
        V = max(V, 1e-6);
        gamma = asin(-w / V);     % Flight path angle (rad) — positive = climbing
        chi   = atan2(v_l, u);    % Heading in horizontal plane (rad)
        varargout{1} = gamma;
        varargout{2} = chi;

    otherwise
        error('[coordinate_transforms] Unknown mode: %s', mode);
end

end
