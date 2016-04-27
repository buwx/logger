CREATE TABLE sensor (
  dateTime    BIGINT NOT NULL,
  sensor      CHAR(1) NOT NULL,
  data        VARCHAR(80),
  description VARCHAR(80),
  INDEX (datetime),
  INDEX (sensor)
);
